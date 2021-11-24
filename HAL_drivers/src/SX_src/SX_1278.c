/*
 * sx_1278.c
 *
 *  Created on: Jan , 2021
 *      Author: Hend
 */


#include <string.h>
#include <SX_1278.h>



//  RF module:           sx1278
//  FSK:
//  Carry Frequency:     434MHz
//  Bit Rate:            1.2Kbps/2.4Kbps/4.8Kbps/9.6Kbps
//  Tx Power Output:     20dbm/17dbm/14dbm/11dbm
//  Frequency Deviation: +/-35KHz
//  Receive Bandwidth:   83KHz
//  Coding:              NRZ
//  Packet Format:       0x5555555555+0xAA2DD4+"Mark1 Lora sx1278" (total: 29 bytes)
//  LoRa:
//  Carry Frequency:     434MHz
//  Spreading Factor:    6/7/8/9/10/11/12
//  Tx Power Output:     20dbm/17dbm/14dbm/11dbm
//  Receive Bandwidth:   7.8KHz/10.4KHz/15.6KHz/20.8KHz/31.2KHz/41.7KHz/62.5KHz/125KHz/250KHz/500KHz
//  Coding:              NRZ
//  Packet Format:       "Mark1 Lora sx1278" (total: 21 bytes)
//  Tx Current:          about 120mA  (RFOP=+20dBm,typ.)
//  Rx Current:          about 11.5mA  (typ.)


/*******************************************************************************
 * Global variables
 *******************************************************************************/


u8 key_flag;

u16 SysTime = 0;
u8 operation_flag = 0x00;
u8	key1_count = 0x00;
u8	mode = 0x01;//lora mode
u8	Freq_Sel = 0x00;//433M
u8	Power_Sel = 0x00;//
u8	Lora_Rate_Sel = 0x06;//
u8	BandWide_Sel = 0x07;
u8	Fsk_Rate_Sel = 0x00;

u8  TxData[64];
u8 RxData[64];

u8 RxLength = 21;

const u8 sx1278FreqTbl[1][3] =
{
  {0x6C, 0x80, 0x00}, //434MHz
};

const u8 sx1278PowerTbl[4] =
{
  0xFF,                   //20dbm
  0xFC,                   //17dbm
  0xF9,                   //14dbm
  0xF6,                   //11dbm
};

const u8 sx1278SpreadFactorTbl[7] =
{
  6,7,8,9,10,11,12
};

const u8 sx1278LoRaBwTbl[10] =
{
//7.8KHz,10.4KHz,15.6KHz,20.8KHz,31.2KHz,41.7KHz,62.5KHz,125KHz,250KHz,500KHz
  0,1,2,3,4,5,6,7,8,9
};



void sx1278_Config(void);

void sx1278_Standby(void)
{
  SPI_Write(LR_RegOpMode,0x09);                              		//Standby//(((Low Frequency Mode)))or high
}

void sx1278_Sleep(void)
{
  SPI_Write(LR_RegOpMode,0x08);                              		//Sleep//Low Frequency Mode

}

//LoRa mode
void sx1278_EntryLoRa(void)
{
  SPI_Write(LR_RegOpMode,0x88);                                    //Low Frequency Mode & lora mode

}

void sx1278_LoRaClearIrq(void)
{
  SPI_Write(LR_RegIrqFlags,0xFF);
}

void sx1278_LoRaEntryRx(void)
{
  u8 addr;

  sx1278_Config();                                           //setting base parameter
  SPI_Write(REG_LR_PADAC,0x84);                              //Normal high power and Rx
  SPI_Write(LR_RegHopPeriod,0xFF);                          //Symbol periods between frequency hops. (0 = disabled). 1st hop always happen after the 1st header symbol
  SPI_Write(REG_LR_DIOMAPPING1,0x01);                       //DIO0=00, DIO1=00, DIO2=00, DIO3=01 page 46

  SPI_Write(LR_RegIrqFlagsMask,0x3F);                       //Open RxDone interrupt & Timeout in DIO0 &DIO1
  sx1278_LoRaClearIrq();

  SPI_Write(LR_RegPayloadLength,RxLength);                       //RegPayloadLength  21byte(this register must define when the data long of one byte in SF is 6)

  addr = SPI_Read(LR_RegFifoRxBaseAddr);           		  //Read RxBaseAddr
  SPI_Write(LR_RegFifoAddrPtr,addr);                        //RxBaseAddr -> FiFoAddrPtr
  SPI_Write(LR_RegOpMode,0x8d);                        		//Continuous Rx Mode//Low Frequency Mode

	//SysTime = 0;
	while(1)
	{
		if((SPI_Read(LR_RegModemStat)&0x04)==0x04)   //Rx-on going RegModemStat
			break;
		/*
		if(SysTime>=3)
			return 0; */                                             //over time for error
	}

}

u8 sx1278_LoRaReadRSSI(void)
{
  u16 temp=10;
  temp=SPI_Read(LR_RegRssiValue);                  //Read RegRssiValueRssi value
  temp=temp+127-137;                                       //127:Max RSSI, 137:RSSI offset
  return (u8)temp;
}

void sx1278_LoRaRxPacket(void)
{
  u8 i;
  u8 addr;
  u8 packet_size;

  if(Get_NIRQ())
  {
    for(i=0;i<32;i++)
      RxData[i] = 0x00;

    addr = SPI_Read(LR_RegFifoRxCurrentaddr);      //last packet addr
    SPI_Write(LR_RegFifoAddrPtr,addr);                      //RxBaseAddr -> FiFoAddrPtr
    if(sx1278SpreadFactorTbl[Lora_Rate_Sel]==6){           //When SpreadFactor is six will used Implicit Header mode(Excluding internal packet length)
      packet_size=RxLength;
    }
    else{
      packet_size = SPI_Read(LR_RegRxNbBytes);     //Number for received bytes
    }
    SPI_BurstRead(0x00, RxData, packet_size);

    sx1278_LoRaClearIrq();

  }

}


void sx1278_LoRaEntryTx(void)
{
  u8 addr,temp;

  sx1278_Config();                                         //setting base parameter

  SPI_Write(REG_LR_PADAC,0x87);                                   //Tx for 20dBm
  SPI_Write(LR_RegHopPeriod,0x00);                               //RegHopPeriod NO FHSS
  SPI_Write(REG_LR_DIOMAPPING1,0x41);                       //DIO0=01, DIO1=00, DIO2=00, DIO3=01

  sx1278_LoRaClearIrq();
  SPI_Write(LR_RegIrqFlagsMask,0xF7);                       //Open TxDone interrupt
  SPI_Write(LR_RegPayloadLength,21);                       //RegPayloadLength  21byte

  addr = SPI_Read(LR_RegFifoTxBaseAddr);           //RegFiFoTxBaseAddr
  SPI_Write(LR_RegFifoAddrPtr,addr);                        //RegFifoAddrPtr
	//SysTime = 0;
	while(1)
	{
		temp=SPI_Read(LR_RegPayloadLength);
		if(temp==21)
		{
			break;
		}
		/*
		if(SysTime>=3)
			return 0;*/
	}

}

void sx1278_LoRaTxPacket(void)
{


	SPI_BurstWrite(0x00, (u8 *)TxData, strlen((char*)TxData)); //Write data
	SPI_Write(LR_RegOpMode,0x8b);                    //Tx Mode
	while(1)
	{
		if(Get_NIRQ())                      //Packet send over
		{
			SPI_Read(LR_RegIrqFlags);
			sx1278_LoRaClearIrq();                                //Clear irq

			sx1278_Standby();                                     //Entry Standby mode

			break;
		}
	}

}


u8 sx1278_ReadRSSI(void)
{
  u8 temp=0xff;

  temp=SPI_Read(0x11);
  temp>>=1;
  temp=127-temp;                                           //127:Max RSSI
  return temp;
}

void sx1278_Config(void)
{

	sx1278_Sleep();                                      //Change modem mode Must in Sleep mode

	for(int i=0; i<50000;i++);

  //lora mode
	sx1278_EntryLoRa();


	SPI_BurstWrite(LR_RegFrMsb,sx1278FreqTbl[Freq_Sel],3);  //setting frequency parameter

	//setting base parameter
	SPI_Write(LR_RegPaConfig,sx1278PowerTbl[Power_Sel]);             //Setting output power parameter

	SPI_Write(LR_RegOcp,0x0B);                              //RegOcp,Close Ocp
	SPI_Write(LR_RegLna,0x23);                              //RegLNA,High & LNA Enable

	if(sx1278SpreadFactorTbl[Lora_Rate_Sel]==6)           //SFactor=6
	{
		u8 tmp;
		SPI_Write(LR_RegModemConfig1,((sx1278LoRaBwTbl[BandWide_Sel]<<4)+(CR<<1)+0x01));
		//Implicit Enable CRC Enable(0x02) & Error Coding rate 4/5(0x01), 4/6(0x02), 4/7(0x03), 4/8(0x04)

		SPI_Write(LR_RegModemConfig2,((sx1278SpreadFactorTbl[Lora_Rate_Sel]<<4)+(SPI_CRC<<2)+0x03));

		tmp = SPI_Read(0x31); //LoRa detection Optimize for SF6
		tmp &= 0xF8;
		tmp |= 0x05;
		SPI_Write(0x31,tmp);
		SPI_Write(0x37,0x0C); //Detection Threshold
	}
	else
	{
		SPI_Write(LR_RegModemConfig1,((sx1278LoRaBwTbl[BandWide_Sel]<<4)+(CR<<1)+0x00));
		//Explicit Enable CRC Enable(0x02) & Error Coding rate 4/5(0x01), 4/6(0x02), 4/7(0x03), 4/8(0x04)
		SPI_Write(LR_RegModemConfig2,((sx1278SpreadFactorTbl[Lora_Rate_Sel]<<4)+(SPI_CRC<<2)+0x03));
		//SFactor &  LNA gain set by the internal AGC loop
	}
	SPI_Write(LR_RegSymbTimeoutLsb,0xFF);                   //RegSymbTimeoutLsb Timeout = 0x3FF(Max)

	SPI_Write(LR_RegPreambleMsb,0x00);                       //RegPreambleMsb
	SPI_Write(LR_RegPreambleLsb,12);                      //RegPreambleLsb 8+4=12byte Preamble

	SPI_Write(REG_LR_DIOMAPPING2,0x01);                     //RegDioMapping2 DIO5=00, DIO4=01

  sx1278_Standby();                                         //Entry standby mode
}

