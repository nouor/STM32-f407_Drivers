#include "cc1101.h"

void CC1101Read(struct sCC1101PhyInfo *phyInfo,
                unsigned char address,
                unsigned char *buffer,
                unsigned char count)
{
  if (!phyInfo->sleep)
  {
    // Check if this is a status register read. If so, the burst bit must be set
    // to distinguish between a strobe command and a status register address.
    if (address > 0x2F && address < 0x3E)
    {
      address |= CC1101_READ_BURST;
    }
    else
    {
      // Format command (R/W, Burst/Single, Address[5:0]).
      address = address | CC1101_READ_SINGLE;
      if (count > 1)
      {
        address = address | CC1101_READ_BURST;
      }
    }

    SPI_CC_BurstRead(address, buffer, count);
  }
}


void CC1101Write(struct sCC1101PhyInfo *phyInfo,
                unsigned char address,
                const unsigned char *buffer,
                unsigned char count)
{
  if (!phyInfo->sleep)
  {
    // Format command (R/W, Burst/Single, Address[5:0]).
    if (count > 1)
    {
      address = address | CC1101_WRITE_BURST;
    }

    SPI_CC_BurstWrite(address, buffer, count);
  }

}

/* CC1101GetChipPartnum - get the hardware part number.
 *
 *  phyInfo CC1101 interface state information used by the interface for all chip interaction.
 * The part number flashed into the chip.
 */
#define CC1101GetChipPartnum(phyInfo)\
  CC1101GetRegister(phyInfo, CC1101_PARTNUM)

/**
 *  CC1101GetChipVersion - get the hardware part version number.
 *
 *     phyInfo CC1101 interface state information used by the interface
 *                    for all chip interaction.
 *
 *      The part version flashed into the chip.
 */
#define CC1101GetChipVersion(phyInfo)\
  CC1101GetRegister(phyInfo, CC1101_VERSION)

/**
 *  CC1101GetRegisterWithSpiSyncProblem - get register value that is affected by
 *  the SPI synchronization issue. The register may be read up to four times,
 *  but is considered valid once two reads in a row yield the same result.
 *    The workaround suggested by the Errata Notes is so read the register up
 *  to four times while comparing the last two reads with one another. Once
 *  there is a match, you have read a valid value.
  */

unsigned char CC1101GetRegisterWithSpiSyncProblem(struct sCC1101PhyInfo *phyInfo,
                                                  unsigned char address)
{
	unsigned char i;
	unsigned char state[4];

	for (i = 0; i < 4; i++)
	{
    CC1101Read(phyInfo, address, (unsigned char*)&state[i], 1);
    // If two consecutive reads yield the same result, then we are guaranteed
    // that the value is valid; no need to continue further...
    if ((i > 0) && (state[i] == state[i-1]))
		{
			break;
		}
	}

	return state[i];
}

/**
 *  CC1101TimeoutEvent - check for timeout event. An incrementing tick must be
 *  supplied. When the tick exceeds the MAX_TIMEOUT value, a timeout event has
 *  occurred.
 *
 *   tick  Amount of time passed waiting for an event to occur.
 *
 *    @return Status of a timeout event occurring.
 */

bool CC1101TimeoutEvent(unsigned int *tick)
{
  if (*tick < CC1101_MAX_TIMEOUT)
  {
    (*tick)++;
    return 0;
  }

  return 1;
}

/**
 *  CC1101SetAndVerifyState - strobe the CC1101 to go to a certain state. Verify
 *  that the new state matches what was desired.
 *
 *  Note: The strobe command and verfication marc state should match. If they
 *  do not, this function is guaranteed to fail.
 *
 *    @param  phyInfo CC1101 interface state information used by the interface
 *                    for all chip interaction.
 *    @param  command Command strobe that will be issued to the radio and
 *                    verified.
 *    @param  state   Command's associated state (e.g. if the command was to go
 *                    into an IDLE state, then eCC1101MarcStateIdle is the
 *                    desired state).
 *
 *    @return Success of the operation.
 */

bool CC1101SetAndVerifyState(struct sCC1101PhyInfo *phyInfo,
                             unsigned char command,
                             enum eCC1101MarcState state)
{
  unsigned int tick = 0;

  CC1101Strobe(phyInfo, command);
  while (CC1101GetMarcState(phyInfo) != state)
  {
    if (CC1101TimeoutEvent(&tick))
    {
      return 0;
    }
  }
  return 1;
}


bool CC1101Configure(struct sCC1101PhyInfo *phyInfo, const struct sCC1101 *config)
{
  /**
   *  Make sure the radio is in an IDLE state before attempting to configure any
   *  packet handling registers. See Table 28 in the CC1101 User's Guide for
   *  more information (swrs061g).
   */
  if (!CC1101SetAndVerifyState(phyInfo, CC1101_SIDLE, eCC1101MarcStateIdle))
  {
    return 0;
  }

  CC1101Write(phyInfo,
              0x00,
              (unsigned char *)((struct sCC1101*)config),
              sizeof(struct sCC1101)/sizeof(unsigned char));

  return 1;
}


unsigned char CC1101GetRegister(struct sCC1101PhyInfo *phyInfo,
                                unsigned char address)
{
  switch (address)
  {
    /**
     *  Note: In order to allow for efficient continuous reading of the RSSI
     *  value, the CC1101_RSSI case has been commented out. RSSI is not
     *  protected by the fix for the Errata Notes issue. A fix must be performed
     *  outside of this device driver (such as averaging).
     */
    //case CC1101_RSSI:
    case CC1101_FREQEST:
    case CC1101_MARCSTATE:
    case CC1101_RXBYTES:
    case CC1101_TXBYTES:
    case CC1101_WORTIME1:
    case CC1101_WORTIME0:
      return CC1101GetRegisterWithSpiSyncProblem(phyInfo, address);
    default:
      {
        unsigned char value;
        CC1101Read(phyInfo, address, &value, 1);
        return value;
      }
  }
}



void CC1101SetRegister(struct sCC1101PhyInfo *phyInfo,
                       unsigned char address,
                       unsigned char value)
{
  CC1101Write(phyInfo, address, &value, 1);
}

void CC1101ReadRegisters(struct sCC1101PhyInfo *phyInfo,
                         unsigned char address,
                         unsigned char *buffer,
                         unsigned char count)
{
  if (count == 1)
  {
    *buffer = CC1101GetRegister(phyInfo, address);
  }
  else if (count > 1)
  {
    CC1101Read(phyInfo, address, buffer, count);
  }
}


void CC1101WriteRegisters(struct sCC1101PhyInfo *phyInfo,
                          unsigned char address,
                          unsigned char *buffer,
                          unsigned char count)
{
  CC1101Write(phyInfo, address, buffer, count);
}

enum eCC1101Chip CC1101GetChip(struct sCC1101PhyInfo *phyInfo)
{
  volatile unsigned char partNum = CC1101GetChipPartnum(phyInfo);
  volatile unsigned char version = CC1101GetChipVersion(phyInfo) & 0xf;


  if (partNum == CC1101_CHIPPARTNUM && version == CC1101_CHIPVERSION)
	  return eCC1101Chip1101;
  else
     return eCC1101ChipUnknown;
}



unsigned char CC1101ReadRxFifo(struct sCC1101PhyInfo *phyInfo,
                               unsigned char *buffer,
                               unsigned char count)
{
  volatile unsigned char rxBytes = CC1101GetRxFifoCount(phyInfo);

  if (rxBytes < count)
  {
    CC1101Read(phyInfo, CC1101_RXFIFO, buffer, rxBytes);
    return rxBytes;
  }
  else
  {
    CC1101Read(phyInfo, CC1101_RXFIFO, buffer, count);
    return count;
  }
}




void CC1101WriteTxFifo(struct sCC1101PhyInfo *phyInfo,
                       unsigned char *buffer,
                       unsigned char count)
{
  CC1101Write(phyInfo, CC1101_TXFIFO, buffer, count);
}

// -----------------------------------------------------------------------------
// Device state control

void CC1101Strobe(struct sCC1101PhyInfo *phyInfo, unsigned char command)
{
  CC1101Write(phyInfo, (command & 0xBF), NULL, 0);
}

bool CC1101TurnOffCrystalOscillator(struct sCC1101PhyInfo *phyInfo)
{
  // CC1101 must be in an IDLE state before powering down.
  if (!CC1101SetAndVerifyState(phyInfo, CC1101_SIDLE, eCC1101MarcStateIdle))
  {
    return 0;
  }

  CC1101Strobe(phyInfo, CC1101_SXOFF);

  return 1;
}

bool CC1101Calibrate(struct sCC1101PhyInfo *phyInfo)
{
  // Calibrate once radio is in IDLE state.
  if (!CC1101SetAndVerifyState(phyInfo, CC1101_SIDLE, eCC1101MarcStateIdle))
  {
    return 0;
  }

  CC1101Strobe(phyInfo, CC1101_SCAL);

  return 1;
}

bool CC1101Sleep(struct sCC1101PhyInfo *phyInfo)
{
  if (!phyInfo->sleep)
  {

    if (!CC1101SetAndVerifyState(phyInfo, CC1101_SIDLE, eCC1101MarcStateIdle))
    {
      return 0;
    }

    /**
     *  Once the radio is asleep, any CSn toggling will wake the radio up. We have
     *  to assume the radio is going to sleep and must therefore set the state
     *  without the use of CC1101GetMarcState().
     */
    CC1101Strobe(phyInfo, CC1101_SPWD);
    phyInfo->sleep = 1;
  }

  return 1;
}

void CC1101Wakeup(struct sCC1101PhyInfo *phyInfo,
                  const unsigned char agctest,
                  const unsigned char test[3],
                  const unsigned char *paTable,
                  unsigned char paTableSize)
{
  /**
   *  Assumes that the SPI Read/Write implementation automatically handles
   *  asserting CSn for this radio (active low) and has waited for the SPI MISO
   *  pin to go low (CHIP_RDYn). If this is not done, this procedure WILL NOT
   *  WORK CORRECTLY.
   */
  if (phyInfo->sleep)
  {
    // If the radio is coming out of a sleep state, perform a wake up routine to
    // reestablish the initial radio state.
    CC1101Strobe(phyInfo, CC1101_SIDLE);
    phyInfo->sleep = 0;

    /**
     *  The last valid calibration results are maintained so calibration is
     *  still valid after waking up from a low power state. The TX/RX FIFOs have
     *  been flushed. If IOCFGx.GDOx_CFG setting is less that 0x20 and
     *  IOCFGx.GDOx_INV is 0(1), GDO0 and GDO2 pins are hard wired to 0(1), and
     *  GDO1 is hard wired to 1(0), until CSn goes low. The following registers
     *  are not retained and must be rewritten: AGCTEST, TEST2, TEST1, TEST0,
     *  PATABLE(contents of PATABLE are lost except the first byte).
     */
    CC1101Write(phyInfo, CC1101_REG_AGCTEST, &agctest, 1);
    CC1101Write(phyInfo, CC1101_REG_TEST2, test, 3);
    CC1101Write(phyInfo, CC1101_PATABLE, paTable, paTableSize);
  }
}


// -----------------------------------------------------------------------------

static const struct sCC1101 gCC1101Settings = {
  0x2E,               // GDO2 output pin configuration.
  0x2E,               // GDO1 output pin configuration.
  0x06,               // GDO0 output pin configuration.
  0x07,               // RXFIFO and TXFIFO thresholds.
  0xD3,               // Sync word, high byte
  0x91,               // Sync word, low byte
  0xFF,               // Packet length.
  0x0C,               // Packet automation control.
  0x05,               // Packet automation control.
  0x00,               // Device address.
  0x00,               // Channel number.
  0x0A,               // Frequency synthesizer control.
  0x00,               // Frequency synthesizer control.
  0x22,               // Frequency control word, high byte.
  0xB6,               // Frequency control word, middle byte.
  0x27,               // Frequency control word, low byte.
  0x35,               // Modem configuration.
  0x83,               // Modem configuration.
  0x03,               // Modem configuration.
  0x21,               // Modem configuration.
  0xEE,               // Modem configuration.
  0x65,               // Modem deviation setting (when FSK modulation is enabled).
  0x07,               // Main Radio Control State Machine configuration.
  0x00,               // Main Radio Control State Machine configuration.
  0x18,               // Main Radio Control State Machine configuration.
  0x16,               // Frequency Offset Compensation Configuration.
  0x6C,               // Bit synchronization Configuration.
  0x07,               // AGC control.
  0x40,               // AGC control.
  0x91,               // AGC control.
  0x87,               // High byte Event 0 timeout
  0x6B,               // Low byte Event 0 timeout
  0xF8,               // Wake On Radio control
  0x57,               // Front end RX configuration.
  0x10,               // Front end RX configuration.
  0xE9,               // Frequency synthesizer calibration.
  0x2A,               // Frequency synthesizer calibration.
  0x00,               // Frequency synthesizer calibration.
  0x1F,               // Frequency synthesizer calibration.
  0x41,               // RC oscillator configuration
  0x00,               // RC oscillator configuration
  0x59,               // Frequency synthesizer calibration control
  0x7F,               // Production test
  0x3C,               // AGC test
  0x88,               // Various test settings.
  0x35,               // Various test settings.
  0x09                // Various test settings.
};
const struct sCC1101 *gConfig = &gCC1101Settings;
struct sCC1101PhyInfo gPhyInfo;
//static unsigned char gPaTable[8] = { 0xC0 }; // 10db at 433MHZ


#ifdef TEST_CC1101_TRANSMITTER
static unsigned char txData[8] = { 0x07, 0x00, 0x00, 'H', 'e', 'l', 'l', '0'};
#endif

#ifdef TEST_CC1101_RECEIVER
static unsigned char rxData[8];
volatile static bool packetAvailable = 0;
#endif

// -----------------------------------------------------------------------------

/**
 *  Transmitter - operator strictly as a transmitting unit. Send out TX data
 *  periodically (using a software delay) while incrementing a sequence counter.
 *  On every transmission, toggle an LED.
 *
 *  Note: The transmitter uses a polling approach to determine when TX data has
 *  been sent. An interrupt-driven approach is demonstrated by the receiver.
 */
void Transmitter(void);
void Receiver(void);
/*int main (void){
	 // Test: configure with default settings.
	  CC1101Configure(&gPhyInfo, &gCC1101Settings);

	  // Test: set the PA table to an initial power level.
	  CC1101WriteRegisters(&gPhyInfo, CC1101_PATABLE, gPaTable, 8);
}*/
/*
   *  Test polling transmit/receive operations on two different units. One unit
   *  is designated the receiver while the other is the transmitter. This method
   *  can be modified to use an interrupt-driven approach. Toggle an LED when
   *  done receiving and transmitting.
   */

  #if defined(TEST_CC1101_RECEIVER)
  Receiver();
  #elif defined(TEST_CC1101_TRANSMITTER)
  Transmitter();
  #endif

#ifdef TEST_CC1101_TRANSMITTER
void Transmitter()
{
  while (true)
  {
    unsigned int i;

    // Increase the sequence number on every transmission. This is mainly used
    // to display differentiation between messages (demo purposes).
    txData[2]++;

    // Add delay between transmits for visual effect
    for (i = 0; i < 60000; i++);

    // Put the radio into an IDLE state.
    CC1101Idle(&gPhyInfo);

    // Flush the TX FIFO and then write new data to it. Transmit.
    CC1101FlushTxFifo(&gPhyInfo);
    CC1101WriteTxFifo(&gPhyInfo, txData, sizeof(txData)/sizeof(unsigned char));
    CC1101Transmit(&gPhyInfo);

  }
}
#endif

#ifdef TEST_CC1101_RECEIVER
void Receiver()
{

  // Enable GDO interrupt and global interrupts.
  CC1101GdoEnable(gPhyInfo.gdo[0]);
  __enable_interrupt();

  while (true)
  {
    // Put the radio into an IDLE state.
    CC1101Idle(&gPhyInfo);

    // Flush the RX FIFO to prepare it for the next RF packet and turn on the
    // receiver.
    CC1101FlushRxFifo(&gPhyInfo);
    CC1101ReceiverOn(&gPhyInfo);


    // Read the data from the RX FIFO.
    CC1101ReadRxFifo(&gPhyInfo, rxData, sizeof(rxData)/sizeof(unsigned char));

    packetAvailable = 0;
  }
}

#endif
