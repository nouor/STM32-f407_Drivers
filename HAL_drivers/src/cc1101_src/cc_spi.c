/*
 * CC_spi.c
 *
 *  Created on: Jan 14, 2021
 *      Author: Nour
 */


#include "cc_spi.h"


//SPI_TypeDef *CC_spi;

//CC_spi = &SPI1;

#define CC_spi  ((SPI_TypeDef *) SPI1_BASE)

/**********************************************************
**Name:     SPICmd
**Function: SPI Write one byte
**Input:    data
**Output:   none
**note:     use for burst mode
**********************************************************/
void SPI_CC_Cmd(uint8_t data)
{


	//NSS_L();
	SPI_SSOEConfig(CC_spi,DISABLE);

	SPI_SendData(CC_spi,&data,sizeof(data));

}

/**********************************************************
**Name:     SPI_Read8bit
**Function: SPI Read one byte
**Input:    None
**Output:   result byte
**Note:     use for burst mode
**********************************************************/
uint8_t SPI_CC_Read8bit(void)
{
 uint8_t Rd_data = 0;

 //NSS_L();
 SPI_SSOEConfig(CC_spi,DISABLE);
 SPI_ReceiveData(CC_spi,&Rd_data,sizeof(Rd_data));


  return(Rd_data);
}

/**********************************************************
**Name:     SPI_Read
**Function: SPI Read CMD
**Input:    adr -> address for read
**Output:   None
**********************************************************/
uint8_t SPI_CC_Read(uint8_t adr)
{
  uint8_t tmp;
  SPI_CC_Cmd(adr);                                         //Send address first
  tmp = SPI_CC_Read8bit();


  //NSS_H();

  SPI_SSOEConfig(CC_spi,ENABLE);

  return(tmp);
}

/**********************************************************
**Name:     SPIWrite
**Function: SPI Write CMD
**Input:    uint8_t address & uint8_t data
**Output:   None
**********************************************************/
void SPI_CC_Write(uint8_t adr, uint8_t data)
{


	//NSS_L();
	SPI_SSOEConfig(CC_spi,DISABLE);

	SPI_CC_Cmd(adr|0x80);
	SPI_CC_Cmd(data);

	SPI_SSOEConfig(CC_spi,ENABLE);

	//NSS_H();
}
/**********************************************************
**Name:     SPIBurstRead
**Function: SPI burst read mode
**Input:    adr-----address for read
**          ptr-----data buffer point for read
**          length--how many bytes for read
**Output:   None
**********************************************************/
void SPI_CC_BurstRead(uint8_t adr, uint8_t *p_data, uint8_t length)
{
  uint8_t i;
  if(length<=1)                                            //length must more than one
    return;
  else
  {

	//NSS_L();
	SPI_SSOEConfig(CC_spi,DISABLE);
    SPI_CC_Cmd(adr);
    for(i=0;i<length;i++)
    	p_data[i] = SPI_CC_Read8bit();
    //nSEL_H();
    //NSS_H();
    SPI_SSOEConfig(CC_spi,ENABLE);
  }
}

/**********************************************************
**Name:     SPIBurstWrite
**Function: SPI burst write mode
**Input:    adr-----address for write
**          ptr-----data buffer point for write
**          length--how many bytes for write
**Output:   none
**********************************************************/
void SPI_CC_BurstWrite(uint8_t adr, const uint8_t *p_data, uint8_t length)
{
  uint8_t i;

  if(length<=1)                                            //length must more than one
    return;
  else
  {

	//NSS_L();
	SPI_SSOEConfig(CC_spi,DISABLE);
    SPI_CC_Cmd(adr|0x80);
    for(i=0;i<length;i++)
	SPI_CC_Cmd(p_data[i]);

    SPI_SSOEConfig(CC_spi,ENABLE);
    //NSS_H();
  }
}






