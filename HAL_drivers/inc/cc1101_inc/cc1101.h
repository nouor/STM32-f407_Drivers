#ifndef CC1101_H
#define CC1101_H
#ifndef bool
#define bool unsigned char
#endif
#ifndef NULL
#define NULL  (void*)0
#endif

#include "stm32f407xx_spi_driver.h"
#include "cc_spi.h"
#include "stm32f4xx.h"
#include "stm32f407xx.h"
#include "our_stm32f407xx.h"

// Supported chip information


#define CC1101_CHIPPARTNUM    0x00
#define CC1101_CHIPVERSION    0x04


// Command strobes
#define CC1101_SRES           0x30 // Reset chip
#define CC1101_SFSTXON        0x31 // Enable and calibrate frequency synthesizer
#define CC1101_SXOFF          0x32 // Turn off crystal oscillator
#define CC1101_SCAL           0x33 // Calibrate frequency synthesizer and turn it off
#define CC1101_SRX            0x34 // Enable rx
#define CC1101_STX            0x35 // In IDLE state: enable TX
#define CC1101_SIDLE          0x36 // Exit RX/TX, turn off frequency synthesizer
// Note: CC1101 datasheet skips register at 0x37u. Use is unavailable.
#define CC1101_SWOR           0x38 // Start automatic RX polling sequence
#define CC1101_SPWD           0x39 // Enter power down mode when CSn goes high
#define CC1101_SFRX           0x3A // Flush the RX FIFO buffer
#define CC1101_SFTX           0x3B // Flush the TX FIFO buffer
#define CC1101_SWORRST        0x3C // Reset real time clock to Event1 value
#define CC1101_SNOP           0x3D // No operation

// Status registers
#define CC1101_PARTNUM        0x30 // Chip ID
#define CC1101_VERSION        0x31 // Chip ID
#define CC1101_FREQEST        0x32 // Frequency offset estimate from demodulator
#define CC1101_LQI            0x33 // Demodulator estimate for link quality
#define CC1101_RSSI           0x34 // Received signal strength indication
#define CC1101_MARCSTATE      0x35 // Main radio control state machine state
#define CC1101_WORTIME1       0x36 // High byte of WOR time
#define CC1101_WORTIME0       0x37 // Low byte of WOR time
#define CC1101_PKTSTATUS      0x38 // Current GDOx status and packet status
#define CC1101_VCO_VC_DAC     0x39 // Current setting from PLL calibration module
#define CC1101_TXBYTES        0x3A // Underflow and number of bytes
#define CC1101_RXBYTES        0x3B // Overflow and number of bytes
#define CC1101_RCCTRL1_STATUS 0x3C // Last RC oscillator calibration result
#define CC1101_RCCTRL0_STATUS 0x3D // Last RC oscillator calibration result

// PA table register
#define CC1101_PATABLE        0x3E // RF output power level

// RX/TX FIFO registers
#define CC1101_RXFIFO         0x3F // Receive FIFO buffer (read-only)
#define CC1101_TXFIFO         0x3F // Transmit FIFO buffer (write-only)

// -----------------------------------------------------------------------------

// Configuration register addresses
#define CC1101_REG_IOCFG2         0x00 // GDO2 Output Pin Configuration
#define CC1101_REG_IOCFG1         0x01 // GDO1 Output Pin Configuration
#define CC1101_REG_IOCFG0         0x02 // GDO0 Output Pin Configuration
#define CC1101_REG_FIFOTHR        0x03 // RX FIFO and TX FIFO Thresholds
#define CC1101_REG_SYNC1          0x04 // Sync Word, High Byte
#define CC1101_REG_SYNC0          0x05 // Sync Word, Low Byte
#define CC1101_REG_PKTLEN         0x06 // Packet Length
#define CC1101_REG_PKTCTRL1       0x07 // Packet Automation Control
#define CC1101_REG_PKTCTRL0       0x08 // Packet Automation Control
#define CC1101_REG_ADDR           0x09 // Device Address
#define CC1101_REG_CHANNR         0x0A // Channel Number
#define CC1101_REG_FSCTRL1        0x0B // Frequency Synthesizer Control
#define CC1101_REG_FSCTRL0        0x0C // Frequency Synthesizer Control
#define CC1101_REG_FREQ2          0x0D // Frequency Control Word, High Byte
#define CC1101_REG_FREQ1          0x0E // Frequency Control Word, Middle Byte
#define CC1101_REG_FREQ0          0x0F // Frequency Control Word, Low Byte
#define CC1101_REG_MDMCFG4        0x10 // Modem Configuration
#define CC1101_REG_MDMCFG3        0x11 // Modem Configuration
#define CC1101_REG_MDMCFG2        0x12 // Modem Configuration
#define CC1101_REG_MDMCFG1        0x13 // Modem Configuration
#define CC1101_REG_MDMCFG0        0x14 // Modem Configuration
#define CC1101_REG_DEVIATN        0x15 // Modem Deviation Setting
#define CC1101_REG_MCSM2          0x16 // Main Radio Control State Machine Configuration
#define CC1101_REG_MCSM1          0x17 // Main Radio Control State Machine Configuration
#define CC1101_REG_MCSM0          0x18 // Main Radio Control State Machine Configuration
#define CC1101_REG_FOCCFG         0x19 // Frequency Offset Compensation Configuration
#define CC1101_REG_BSCFG          0x1A // Bit Synchronization Configuration
#define CC1101_REG_AGCCTRL2       0x1B // AGC Control
#define CC1101_REG_AGCCTRL1       0x1C // AGC Control
#define CC1101_REG_AGCCTRL0       0x1D // AGC Control
#define CC1101_REG_WOREVT1        0x1E // High Byte Event0 Timeout
#define CC1101_REG_WOREVT0        0x1F // Low Byte Event0 Timeout
#define CC1101_REG_WORCTRL        0x20 // Wake On Radio Control
#define CC1101_REG_FREND1         0x21 // Front End RX Configuration
#define CC1101_REG_FREND0         0x22 // Front End TX Configuration
#define CC1101_REG_FSCAL3         0x23 // Frequency Synthesizer Calibration
#define CC1101_REG_FSCAL2         0x24 // Frequency Synthesizer Calibration
#define CC1101_REG_FSCAL1         0x25 // Frequency Synthesizer Calibration
#define CC1101_REG_FSCAL0         0x26 // Frequency Synthesizer Calibration
#define CC1101_REG_RCCTRL1        0x27 // RC Oscillator Configuration
#define CC1101_REG_RCCTRL0        0x28 // RC Oscillator Configuration
#define CC1101_REG_FSTEST         0x29 // Frequency Synthesizer Calibration Control
#define CC1101_REG_PTEST          0x2A // Production Test
#define CC1101_REG_AGCTEST        0x2B // AGC Test
#define CC1101_REG_TEST2          0x2C // Various Test Settings
#define CC1101_REG_TEST1          0x2D // Various Test Settings
#define CC1101_REG_TEST0          0x2E // Various Test Settings


// Configuration register field masks
// IOCFG2
#define CC1101_GDO2_INV                   0x40
#define CC1101_GDO2_CFG                   0x3F
// IOCFG1
#define CC1101_GDO1_DS                    0x80
#define CC1101_GDO1_INV                   0x40
#define CC1101_GDO1_CFG                   0x3F
// IOCFG0
#define CC1101_GDO0_TEMP_SENSOR_ENABLE    0x80
#define CC1101_GDO0_INV                   0x40
#define CC1101_GDO0_CFG                   0x3F
// FIFOTHR
#define CC1101_ADC_RETENTION              0x40
#define CC1101_CLOSE_IN_RX                0x30
#define CC1101_FIFO_THR                   0x0F
// SYNC1
#define CC1101_SYNC_MSB                   0xFF
// SYNC0
#define CC1101_SYNC_LSB                   0xFF
// PKTLEN
#define CC1101_PACKET_LENGTH              0xFF
// PKTCTRL1
#define CC1101_PQT                        0xE0
#define CC1101_CRC_AUTOFLUSH              0x08
#define CC1101_APPEND_STATUS              0x04
#define CC1101_ADR_CHK                    0x03
// PKTCTRL0
#define CC1101_WHITE_DATA                 0x40
#define CC1101_PKT_FORMAT                 0x30
#define CC1101_CRC_EN                     0x04
#define CC1101_LENGTH_CONFIG              0x03
// ADDR
#define CC1101_DEVICE_ADDR                0xFF
// CHANNR
#define CC1101_CHANNR_CHAN                0xFF
// FSCTRL1
#define CC1101_FREQ_IF                    0x1F
// FSCTRL0
#define CC1101_FREQOFF                    0xFF
// FREQ2
#define CC1101_FREQ_23_22                 0xC0
#define CC1101_FREQ_21_16                 0x3F
// FREQ1
#define CC1101_FREQ_15_8                  0xFF
// FREQ0
#define CC1101_FREQ_7_0                   0xFF
// MDMCFG4
#define CC1101_CHANBW_E                   0xC0
#define CC1101_CHANBW_M                   0x30
#define CC1101_DRATE_E                    0x0F
// MDMCFG3
#define CC1101_DRATE_M                    0xFF
// MDMCFG2
#define CC1101_DEM_DCFILT_OFF             0x80
#define CC1101_MOD_FORMAT                 0x70
#define CC1101_MANCHESTER_EN              0x08
#define CC1101_SYNC_MODE                  0x07
// MDMCFG1
#define CC1101_FEC_EN                     0x80
#define CC1101_NUM_PREAMBLE               0x70
#define CC1101_CHANSPC_E                  0x03
// MDMCFG0
#define CC1101_CHANSPC_M                  0xFF
// DEVIATN
#define CC1101_DEVIATION_E                0x70
#define CC1101_DEVIATION_M                0x07
// MCSM2
#define CC1101_RX_TIME_RSSI               0x10
#define CC1101_RX_TIME_QUAL               0x08
#define CC1101_RX_TIME                    0x07
// MCSM1
#define CC1101_CCA_MODE                   0x30
#define CC1101_RXOFF_MODE                 0x0C
#define CC1101_TXOFF_MODE                 0x03
// MCSM0
#define CC1101_FS_AUTOCAL                 0x30
#define CC1101_PO_TIMEOUT                 0x0C
#define CC1101_PIN_CTRL_EN                0x02
#define CC1101_XOSC_FORCE_ON              0x01
// FOCCFG
#define CC1101_FOC_BS_CS_GATE             0x20
#define CC1101_FOC_PRE_K                  0x18
#define CC1101_FOC_POST_K                 0x04
#define CC1101_FOC_LIMIT                  0x03
// BSCFG
#define CC1101_BS_PRE_K                   0xC0
#define CC1101_BS_PRE_KP                  0x30
#define CC1101_BS_POST_K                  0x08
#define CC1101_BS_POST_KP                 0x04
#define CC1101_BS_LIMIT                   0x03
// AGCCTRL2
#define CC1101_MAX_DVGA_GAIN              0xC0
#define CC1101_MAX_LNA_GAIN               0x38
#define CC1101_MAGN_TARGET                0x07
// AGCCTRL1
#define CC1101_AGC_LNA_PRIORITY           0x40
#define CC1101_CARRIER_SENSR_REL_THR      0x30
#define CC1101_CARRIER_SENSE_ABS_THR      0x0F
// AGCCTRL0
#define CC1101_HYST_LEVEL                 0xC0
#define CC1101_WAIT_TIME                  0x30
#define CC1101_AGC_FREEZE                 0x0C
#define CC1101_FILTER_LENGTH              0x03
// WOREVT1
#define CC1101_EVENT0_15_8                0xFF
// WOREVT0
#define CC1101_EVENT0_7_0                 0xFF
// WORCTRL
#define CC1101_RC_PD                      0x80
#define CC1101_EVENT1                     0x70
#define CC1101_RC_CAL                     0x08
#define CC1101_WOR_RES                    0x03
// FREND1
#define CC1101_LNA_CURRENT                0xC0
#define CC1101_LNA2MIX_CURRENT            0x30
#define CC1101_LODIV_BUF_CURRENT          0x0C
#define CC1101_MIX_CURRENT                0x03
// FREND0
#define CC1101_LODIV_BUF_CURRENT_TX       0x30
#define CC1101_PA_POWER                   0x07
// FSCAL3
#define CC1101_FSCAL3_7_6                 0xC0
#define CC1101_CHP_CURR_CAL_EN            0x30
#define CC1101_FSCAL3_3_0                 0x0F
// FSCAL2
#define CC1101_VCO_CORE_H_EN              0x20
#define CC1101_FSCAL2_7_0                 0x1F
// FSCAL1
#define CC1101_FSCAL1_7_0                 0x3F
// FSCAL0
#define CC1101_FSCAL0_7_0                 0x7F
// RCCTRL1
#define CC1101_RCCTRL_1                   0x7F
// RCCTRL0
#define CC1101_RCCTRL_0                   0x7F
// FSTEST
#define CC1101_FSTEST_7_0                 0xFF
// PTEST
#define CC1101_PTEST_7_0                  0xFF
// AGCTEST
#define CC1101_AGCTEST_7_0                0xFF
// TEST2
#define CC1101_TEST2_7_0                  0xFF
// TEST1
#define CC1101_TEST1_7_0                  0xFF
// TEST0
#define CC1101_TEST0_7_2                  0xFC
#define CC1101_VCO_SEL_CAL_EN             0x02
#define CC1101_TEST0_0                    0x01

// Status register field masks
// PARTNUM
#define CC1101_PARTNUM_7_0                0xFF
// VERSION
#define CC1101_VERSION_7_0                0xFF
// FREQOFF_EST
#define CC1101_FREQOFF_EST                0xFF
// LQI
#define CC1101_CRC_OK                     0x80
#define CC1101_LQI_EST                    0x7F
// RSSI
#define CC1101_RSSI_7_0                   0xFF
// MARC_STATE
#define CC1101_MARC_STATE                 0x1F
// WORTIME1
#define CC1101_TIME_15_8                  0xFF
// WORTIME0
#define CC1101_TIME_7_0                   0xFF
// PKTSTATUS
#define CC1101_PKTSTATUS_CRC_OK           0x80
#define CC1101_PKSTATUS_CS                0x40
#define CC1101_PKTSTATUS_PQT_REACHED      0x20
#define CC1101_PKTSTATUS_CCA              0x10
#define CC1101_PKSTATUS_SFD               0x08
#define CC1101_PKTSTATUS_GDO2             0x04
#define CC1101_PKTSTATUS_GDO0             0x01
// VCO_VC_DAC
#define CC1101_VCO_VC_DAC_7_0             0xFF
// TXBYTES
#define CC1101_TXFIFO_UNDERFLOW           0x80
#define CC1101_NUM_TXBYTES                0x7F
// RXBYTES
#define CC1101_RXFIFO_OVERFLOW            0x80
#define CC1101_NUM_RXBYTES                0x7F
// RCCTRL1_STATUS
#define CC1101_RCCTRL1_STATUS_7_0         0xFF
// RCCTRL0_STATUS
#define CC1101_RCCTRL0_STATUS_7_0         0xFF

// Burst/single access masks
#define CC1101_WRITE_SINGLE               0x00
#define CC1101_WRITE_BURST                0x40
#define CC1101_READ_SINGLE                0x80
#define CC1101_READ_BURST                 0xC0

// -----------------------------------------------------------------------------

// Physical FIFO size
#define CC1101_RXFIFO_SIZE        64 // Receive hardware FIFO absolute size
#define CC1101_TXFIFO_SIZE        64 // Transmit hardware FIFO absolute size

// Maximum timeout error ticks
#define CC1101_MAX_TIMEOUT        2000
// -----------------------------------------------------------------------------

/**
 *  eCC1101Error - error codes issued by the device driver.
 */
enum eCC1101Error
{
  eCC1101ErrorTimeout     = 0x01,
  eCC1101ErrorSleep       = 0x02
};
enum eCC1101Chip
{
  eCC1101ChipUnknown    = 0,
  eCC1101Chip1101       = 1,
};

/**
 *  eCC1101MarcState - all possible MARCSTATEs that the radio state machine can
 *  traverse.
 */
enum eCC1101MarcState
{
  eCC1101MarcStateSleep             = 0x00,
  eCC1101MarcStateIdle              = 0x01,
  eCC1101MarcStateXOff              = 0x02,
  eCC1101MarcStateVcoon_mc          = 0x03,
  eCC1101MarcStateRegon_mc          = 0x04,
  eCC1101MarcStateMancal            = 0x05,
  eCC1101MarcStateVcoon             = 0x06,
  eCC1101MarcStateRegon             = 0x07,
  eCC1101MarcStateStartcal          = 0x08,
  eCC1101MarcStateBwboost           = 0x09,
  eCC1101MarcStateFs_lock           = 0x0A,
  eCC1101MarcStateIfadcon           = 0x0B,
  eCC1101MarcStateEndcal            = 0x0C,
  eCC1101MarcStateRx                = 0x0D,
  eCC1101MarcStateRx_end            = 0x0E,
  eCC1101MarcStateRx_rst            = 0x0F,
  eCC1101MarcStateTxrx_switch       = 0x10,
  eCC1101MarcStateRxfifo_overflow   = 0x11,
  eCC1101MarcStateFstxon            = 0x12,
  eCC1101MarcStateTx                = 0x13,
  eCC1101MarcStateTx_end            = 0x14,
  eCC1101MarcStateRxtx_switch       = 0x15,
  eCC1101MarcStateTxfifo_underflow  = 0x16,
  eCC1101MarcStateUnknown           = 0xFF
};

/**
 *  eCC1101GdoState - possible states that the GDOx line can be in.
 */
enum eCC1101GdoState
{
  eCC1101GdoStateWaitForAssert    = 0x00,
  eCC1101GdoStateWaitForDeassert  = 0x01
};

// -----------------------------------------------------------------------------


struct sCC1101
{
  unsigned char iocfg2;   // GDO2 output pin configuration
  unsigned char iocfg1;   // GDO1 output pin configuration
  unsigned char iocfg0;   // GDO0 output pin configuration
  unsigned char fifothr;  // RXFIFO and TXFIFO thresholds
  unsigned char sync1;    // Sync word, high byte
  unsigned char sync0;    // Sync word, low byte
  unsigned char pktlen;   // Packet length
  unsigned char pktctrl1; // Packet automation control
  unsigned char pktctrl0; // Packet automation control
  unsigned char addr;     // Device address
  unsigned char channr;   // Channel number
  unsigned char fsctrl1;  // Frequency synthesizer control
  unsigned char fsctrl0;  // Frequency synthesizer control
  unsigned char freq2;    // Frequency control word, high byte
  unsigned char freq1;    // Frequency control word, middle byte
  unsigned char freq0;    // Frequency control word, low byte
  unsigned char mdmcfg4;  // Modem configuration 4
  unsigned char mdmcfg3;  // Modem configuration 3
  unsigned char mdmcfg2;  // Modem configuration 2
  unsigned char mdmcfg1;  // Modem configuration 1
  unsigned char mdmcfg0;  // Modem configuration 0
  unsigned char deviatn;  // Modem deviation setting
  unsigned char mcsm2;    // Main radio control state machine configuration
  unsigned char mcsm1;    // Main radio control state machine configuration
  unsigned char mcsm0;    // Main radio control state machine configuration
  unsigned char foccfg;   // Frequency offset compensation configuration
  unsigned char bscfg;    // Bit synchronization configuration
  unsigned char agcctrl2; // AGC control 2
  unsigned char agcctrl1; // AGC control 1
  unsigned char agcctrl0; // AGC control 0
  unsigned char worevt1;  // High byte event0 timeout
  unsigned char worevt0;  // Low byte event0 timeout
  unsigned char worctrl;  // Wake on radio control
  unsigned char frend1;   // Front end RX configuration
  unsigned char frend0;   // Front end TX configuration
  unsigned char fscal3;   // Frequency synthesizer calibration
  unsigned char fscal2;   // Frequency synthesizer calibration
  unsigned char fscal1;   // Frequency synthesizer calibration
  unsigned char fscal0;   // Frequency synthesizer calibration
  unsigned char rcctrl1;  // RC oscillator configuration
  unsigned char rcctrl0;  // RC oscillator configuration
  unsigned char fstest;   // Frequency synthesizer calibration control
  unsigned char ptest;    // Production test
  unsigned char agctest;  // AGC test
  unsigned char test2;    // Various test settings 2
  unsigned char test1;    // Various test settings 1
  unsigned char test0;    // Various test settings 0
};


struct sCC1101Spi
{
  void(*const Init)(void);
  void(*const Read)(unsigned char, unsigned char, unsigned char);
  void(*const Write)(unsigned char, const unsigned char, unsigned char);
};


struct sCC1101Gdo
{
  void(*const Init)(void);
  bool(*const Event)(volatile const unsigned char);
  void(*const WaitForAssert)(void);
  void(*const WaitForDeassert)(void);
  enum eCC1101GdoState(*const GetState)(void);
  void(*const Enable)(bool);
};


struct sCC1101PhyInfo
{
  struct sCC1101Spi *spi;     // Interface for SPI
  struct sCC1101Gdo *gdo[3];  // Interface for GDOx
  volatile bool sleep;        // Chip sleep flag
};



#define CC1101GetSleepState(phyInfo) (phyInfo->sleep)

// Device configuration and status
void CC1101SpiInit(struct sCC1101PhyInfo *phyInfo,
                   const struct sCC1101Spi *spi,
				   void(*const ErrorHandler)(enum eCC1101Error));

void CC1101GdoInit(struct sCC1101PhyInfo *phyInfo,
                   const struct sCC1101Gdo *gdo[3]);

bool CC1101Configure(struct sCC1101PhyInfo *phyInfo, const struct sCC1101 *config);


unsigned char CC1101GetRegister(struct sCC1101PhyInfo *phyInfo,
                                unsigned char address);

void CC1101SetRegister(struct sCC1101PhyInfo *phyInfo,
                       unsigned char address,
                       unsigned char value);


void CC1101ReadRegisters(struct sCC1101PhyInfo *phyInfo,
                         unsigned char address,
                         unsigned char *buffer,
                         unsigned char count);

void CC1101WriteRegisters(struct sCC1101PhyInfo *phyInfo,
                          unsigned char address,
                          unsigned char *buffer,
                          unsigned char count);

enum eCC1101Chip CC1101GetChip(struct sCC1101PhyInfo *phyInfo);

unsigned char CC1101ReadRxFifo(struct sCC1101PhyInfo *phyInfo,
                               unsigned char *buffer,
                               unsigned char count);

void CC1101WriteTxFifo(struct sCC1101PhyInfo *phyInfo,
                       unsigned char *buffer,
                       unsigned char count);

#define CC1101GetRxFifoCount(phyInfo)\
  CC1101GetRegister(phyInfo, CC1101_RXBYTES)

#define CC1101GetTxFifoCount(phyInfo)\
  CC1101GetRegister(phyInfo, CC1101_TXBYTES)

#define CC1101GetMarcState(phyInfo)\
  (enum eCC1101MarcState)CC1101GetRegister(phyInfo, CC1101_MARCSTATE)

#define CC1101GetRssi(phyInfo)\
  CC1101GetRegister(phyInfo, CC1101_RSSI)


#define CC1101GetLqi(phyInfo)\
  ((CC1101GetRegister(phyInfo, CC1101_LQI)) & CC1101_LQI_EST)

#define CC1101GetCrc(phyInfo)\
  ((CC1101GetRegister(phyInfo, CC1101_LQI)) & CC1101_CRC_OK)

// -----------------------------------------------------------------------------
// Device state control


void CC1101Strobe(struct sCC1101PhyInfo *phyInfo, unsigned char command);


#define CC1101Reset(phyInfo)  CC1101Strobe(phyInfo, CC1101_SRES)


#define CC1101EnableFrequencySynthesizer(phyInfo)\
  CC1101Strobe(phyInfo, CC1101_SFSTXON)

bool CC1101TurnOffCrystalOscillator(struct sCC1101PhyInfo *phyInfo);


bool CC1101Calibrate(struct sCC1101PhyInfo *phyInfo);

#define CC1101ReceiverOn(phyInfo)\
  CC1101Strobe(phyInfo, CC1101_SRX);


#define CC1101Transmit(phyInfo)\
  CC1101Strobe(phyInfo, CC1101_STX);


#define CC1101Idle(phyInfo) CC1101Strobe(phyInfo, CC1101_SIDLE)

#define CC1101StartWakeOnRadio(phyInfo)\
  CC1101Strobe(phyInfo, CC1101_SWOR)

#define CC1101FlushRxFifo(phyInfo)\
  CC1101Strobe(phyInfo, CC1101_SFRX)


#define CC1101FlushTxFifo(phyInfo)\
  CC1101Strobe(phyInfo, CC1101_SFTX)

bool CC1101Sleep(struct sCC1101PhyInfo *phyInfo);

void CC1101Wakeup(struct sCC1101PhyInfo *phyInfo,
                  const unsigned char agctest,
                  const unsigned char test[3],
                  const unsigned char *paTable,
                  unsigned char paTableSize);

#define CC1101ResetWakeOnRadio(phyInfo)\
  CC1101Strobe(phyInfo, CC1101_SWORRST)


#define CC1101Nop(phyInfo)  CC1101Strobe(phyInfo, CC1101_SNOP)

// -----------------------------------------------------------------------------
// Device interrupt

#define CC1101GdoEvent(gdo, event) (gdo->Event(event))

#define CC1101GdoWaitForAssert(gdo) gdo->WaitForAssert()

#define CC1101GdoWaitForDeassert(gdo) gdo->WaitForDeassert()

#define CC1101GdoEnable(gdo)  gdo->Enable(true)

#define CC1101GdoDisable(gdo) gdo->Enable(false)


#define CC1101GdoGetState(gdo)  (gdo->GetState())

#endif
