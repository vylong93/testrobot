/*
 * cc2500.h
 *
 *  Created on: Dec 17, 2014
 *      Author: VyLong
 */


#ifndef CC2500_H_
#define CC2500_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>

#define TI_CCxxx0_COMMON_ADDRESS 0x00

// Configuration Registers
#define TI_CCxxx0_IOCFG2       0x00        // GDO2 output pin configuration
#define TI_CCxxx0_IOCFG1       0x01        // GDO1 output pin configuration
#define TI_CCxxx0_IOCFG0       0x02        // GDO0 output pin configuration
#define TI_CCxxx0_FIFOTHR      0x03        // RX FIFO and TX FIFO thresholds
#define TI_CCxxx0_SYNC1        0x04        // Sync word, high byte
#define TI_CCxxx0_SYNC0        0x05        // Sync word, low byte
#define TI_CCxxx0_PKTLEN       0x06        // Packet length
#define TI_CCxxx0_PKTCTRL1     0x07        // Packet automation control
#define TI_CCxxx0_PKTCTRL0     0x08        // Packet automation control
#define TI_CCxxx0_ADDR         0x09        // Device address
#define TI_CCxxx0_CHANNR       0x0A        // Channel number
#define TI_CCxxx0_FSCTRL1      0x0B        // Frequency synthesizer control
#define TI_CCxxx0_FSCTRL0      0x0C        // Frequency synthesizer control
#define TI_CCxxx0_FREQ2        0x0D        // Frequency control word, high byte
#define TI_CCxxx0_FREQ1        0x0E        // Frequency control word, middle byte
#define TI_CCxxx0_FREQ0        0x0F        // Frequency control word, low byte
#define TI_CCxxx0_MDMCFG4      0x10        // Modem configuration
#define TI_CCxxx0_MDMCFG3      0x11        // Modem configuration
#define TI_CCxxx0_MDMCFG2      0x12        // Modem configuration
#define TI_CCxxx0_MDMCFG1      0x13        // Modem configuration
#define TI_CCxxx0_MDMCFG0      0x14        // Modem configuration
#define TI_CCxxx0_DEVIATN      0x15        // Modem deviation setting
#define TI_CCxxx0_MCSM2        0x16        // Main Radio Cntrl State Machine config
#define TI_CCxxx0_MCSM1        0x17        // Main Radio Cntrl State Machine config
#define TI_CCxxx0_MCSM0        0x18        // Main Radio Cntrl State Machine config
#define TI_CCxxx0_FOCCFG       0x19        // Frequency Offset Compensation config
#define TI_CCxxx0_BSCFG        0x1A        // Bit Synchronization configuration
#define TI_CCxxx0_AGCCTRL2     0x1B        // AGC control
#define TI_CCxxx0_AGCCTRL1     0x1C        // AGC control
#define TI_CCxxx0_AGCCTRL0     0x1D        // AGC control
#define TI_CCxxx0_WOREVT1      0x1E        // High byte Event 0 timeout
#define TI_CCxxx0_WOREVT0      0x1F        // Low byte Event 0 timeout
#define TI_CCxxx0_WORCTRL      0x20        // Wake On Radio control
#define TI_CCxxx0_FREND1       0x21        // Front end RX configuration
#define TI_CCxxx0_FREND0       0x22        // Front end TX configuration
#define TI_CCxxx0_FSCAL3       0x23        // Frequency synthesizer calibration
#define TI_CCxxx0_FSCAL2       0x24        // Frequency synthesizer calibration
#define TI_CCxxx0_FSCAL1       0x25        // Frequency synthesizer calibration
#define TI_CCxxx0_FSCAL0       0x26        // Frequency synthesizer calibration
#define TI_CCxxx0_RCCTRL1      0x27        // RC oscillator configuration
#define TI_CCxxx0_RCCTRL0      0x28        // RC oscillator configuration
#define TI_CCxxx0_FSTEST       0x29        // Frequency synthesizer cal control
#define TI_CCxxx0_PTEST        0x2A        // Production test
#define TI_CCxxx0_AGCTEST      0x2B        // AGC test
#define TI_CCxxx0_TEST2        0x2C        // Various test settings
#define TI_CCxxx0_TEST1        0x2D        // Various test settings
#define TI_CCxxx0_TEST0        0x2E        // Various test settings

// Strobe commands
#define TI_CCxxx0_SRES         0x30        // Reset chip.
#define TI_CCxxx0_SFSTXON      0x31        // Enable/calibrate freq synthesizer
#define TI_CCxxx0_SXOFF        0x32        // Turn off crystal oscillator.
#define TI_CCxxx0_SCAL         0x33        // Calibrate freq synthesizer & disable
#define TI_CCxxx0_SRX          0x34        // Enable RX.
#define TI_CCxxx0_STX          0x35        // Enable TX.
#define TI_CCxxx0_SIDLE        0x36        // Exit RX / TX
#define TI_CCxxx0_SAFC         0x37        // AFC adjustment of freq synthesizer
#define TI_CCxxx0_SWOR         0x38        // Start automatic RX polling sequence
#define TI_CCxxx0_SPWD         0x39        // Enter pwr down mode when CSn goes hi
#define TI_CCxxx0_SFRX         0x3A        // Flush the RX FIFO buffer.
#define TI_CCxxx0_SFTX         0x3B        // Flush the TX FIFO buffer.
#define TI_CCxxx0_SWORRST      0x3C        // Reset real time clock.
#define TI_CCxxx0_SNOP         0x3D        // No operation.

// Status registers
#define TI_CCxxx0_PARTNUM      0x30        // Part number
#define TI_CCxxx0_VERSION      0x31        // Current version number
#define TI_CCxxx0_FREQEST      0x32        // Frequency offset estimate
#define TI_CCxxx0_LQI          0x33        // Demodulator estimate for link quality
#define TI_CCxxx0_RSSI         0x34        // Received signal strength indication
#define TI_CCxxx0_MARCSTATE    0x35        // Control state machine state
#define TI_CCxxx0_WORTIME1     0x36        // High byte of WOR timer
#define TI_CCxxx0_WORTIME0     0x37        // Low byte of WOR timer
#define TI_CCxxx0_PKTSTATUS    0x38        // Current GDOx status and packet status
#define TI_CCxxx0_VCO_VC_DAC   0x39        // Current setting from PLL cal module
#define TI_CCxxx0_TXBYTES      0x3A        // Underflow and # of bytes in TXFIFO
#define TI_CCxxx0_RXBYTES      0x3B        // Overflow and # of bytes in RXFIFO
#define TI_CCxxx0_NUM_RXBYTES  0x7F        // Mask "# of bytes" field in _RXBYTES

// Mask for PKTSTATUS register
#define TI_CCxxx0_CS		   0x40
#define TI_CCxxx0_CCA		   0x10

// Other memory locations
#define TI_CCxxx0_PATABLE      0x3E
#define TI_CCxxx0_TXFIFO       0x3F
#define TI_CCxxx0_RXFIFO       0x3F

// Masks for appended status bytes
#define TI_CCxxx0_RSSI_RX         0        // Position of RSSI byte
#define TI_CCxxx0_LQI_RX          1        // Position of LQI bits
#define TI_CCxxx0_NUMSTATUSBYTES  2        // Number of status bytes appended to each packet
#define TI_CCxxx0_CRC_OK       0x80        // Mask "CRC_OK" bit within LQI byte

// Masks for chip status byte
#define TI_CCxxx0_STATE_MASK	0x70
#define TI_CCxxx0_STATE_TX		0x20

// Definitions to support burst/single access:
#define TI_CCxxx0_WRITE_BURST  0x40
#define TI_CCxxx0_READ_SINGLE  0x80
#define TI_CCxxx0_READ_BURST   0xC0

// PATABLE Tx power
#define TI_CCxxx0_OUTPUT_POWER_1DB	0xFF	// 1 dBm
#define TI_CCxxx0_OUTPUT_POWER_0DB	0xFB	// 0 dBm
#define TI_CCxxx0_OUTPUT_POWER_MINUS_2DB	0xBB	// -2 dBm
#define TI_CCxxx0_OUTPUT_POWER_MINUS_4DB	0xA9	// -4 dBm
#define TI_CCxxx0_OUTPUT_POWER_MINUS_6DB	0x7F	// -6 dBm
#define TI_CCxxx0_OUTPUT_POWER_MINUS_8DB	0x6E	// -8 dBm
#define TI_CCxxx0_OUTPUT_POWER_MINUS_10DB	0x97	// -10 dBm
#define TI_CCxxx0_OUTPUT_POWER_MINUS_12DB	0xC6	// -12 dBm
#define TI_CCxxx0_OUTPUT_POWER_MINUS_14DB	0x8D	// -14 dBm
#define TI_CCxxx0_OUTPUT_POWER_MINUS_16DB	0x55	// -16 dBm
#define TI_CCxxx0_OUTPUT_POWER_MINUS_18DB	0x93	// -18 dBm
#define TI_CCxxx0_OUTPUT_POWER_MINUS_20DB	0x46	// -20 dBm
#define TI_CCxxx0_OUTPUT_POWER_MINUS_22DB	0x81	// -22 dBm
#define TI_CCxxx0_OUTPUT_POWER_MINUS_24DB	0x84	// -24 dBm
#define TI_CCxxx0_OUTPUT_POWER_MINUS_26DB	0xC0	// -26 dBm
#define TI_CCxxx0_OUTPUT_POWER_MINUS_28DB	0x44	// -28 dBm
#define TI_CCxxx0_OUTPUT_POWER_MINUS_30DB	0x50	// -30 dBm
#define TI_CCxxx0_OUTPUT_POWER_MINUS_55DB	0x00	// -55 dBm

// Transmission Parameters
#define TXBUFFERSIZE         64 // Size of buffer to store Tx & Rx FIFO data
#define TXPACKETLEN  		 TXBUFFERSIZE // Number of Tx bytes to send in a single data transmission
#define RFNUMCHANS           5 // Number of channels in frequency hop table

typedef enum
{
	RX_STATUS_SUCCESS,
	RX_STATUS_CRC_ERROR,
	RX_STATUS_FAILED
} e_RxStatus;

// CCxxx0 Manipulation methods
void RfWriteSettings(void);
bool RfSendPacket(uint8_t *txBuffer, uint8_t size);
e_RxStatus RfReceivePacket(uint8_t *rxBuffer);
bool RfTryToGetRxPacket(uint64_t ui64PeriodInUs,
			bool (*pfnDecodePacket)(uint8_t* pRxBuff, va_list argp), ...);
bool RfTryToCaptureRfSignal(uint64_t ui64PeriodInUs,
			bool (*pfnHandler)(va_list argp), ...);

void RfPowerupCSnSequence(void);
void RfResetChip(void);
uint8_t RfReadStatus(void);

uint8_t RfReadReg(uint8_t addr);
void RfReadBurstReg(uint8_t addr, uint8_t *buffer, uint8_t count);
void RfWriteReg(uint8_t addr, uint8_t value);
void RfWriteBurstReg(uint8_t addr, uint8_t *buffer, uint8_t count);

// CCxxx0 Application methods
void initRfModule(bool isEnableInt);
void RfSetChannel(uint8_t chanNum);
void RfSetIdleMode(void);
void RfSetRxMode();
void RfFlushTxFifo();
void RfFlushRxFifo();

#ifdef __cplusplus
}
#endif

#endif /* CC2500_H_ */
