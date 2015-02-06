#ifdef RF_USE_nRF24L01

/* nRF24L01.h
 * Register definitions for manipulating the Nordic Semiconductor
 * nRF24L01+ RF transceiver chipsets.
 *

    Copyright (c) 2007 Stefan Engelke <mbox@stefanengelke.de>
    Some parts copyright (c) 2012 Eric Brundick <spirilis [at] linux dot com>

    Permission is hereby granted, free of charge, to any person 
    obtaining a copy of this software and associated documentation 
    files (the "Software"), to deal in the Software without 
    restriction, including without limitation the rights to use, copy, 
    modify, merge, publish, distribute, sublicense, and/or sell copies 
    of the Software, and to permit persons to whom the Software is 
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be 
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, 
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF 
    MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
    DEALINGS IN THE SOFTWARE.
	
	
	//  ***  modified by VyLong  *** //
 */
 
#ifndef _NRF24L01_H
#define _NRF24L01_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>

typedef struct tag_rf24_iniTypeDef
{
  uint8_t AddressWidth;
  uint8_t Speed;
  uint8_t Power;
  uint8_t Channel;
  uint8_t CrcState;
  uint8_t CrcBytes;
  uint8_t RetransmitDelay;
  uint8_t RetransmitCount;
  uint8_t Features;
  bool InterruptEnable;
  bool LNAGainEnable;
} RF24_InitTypeDef;

#define RF24_DEFAULT_ADDRESS_BYTE0	0xBC	// 1011 1100
#define RF24_DEFAULT_ADDRESS_BYTE1	0xAD	// 1010 1101
#define RF24_DEFAULT_ADDRESS_BYTE2	0x59	// 0101 1001
#define RF24_DEFAULT_ADDRESS_BYTE3	0x37	// 0011 0111
#define RF24_DEFAULT_ADDRESS_BYTE4	0x42	// 0100 0010


// *** Register Map *** //
#define RF24_REG_MASK        0x1F
#define RF24_REG_CONFIG      0x00
#define RF24_REG_CONFIG_MASK 0x7F
#define RF24_REG_EN_AA       0x01
#define RF24_REG_EN_RXADDR   0x02
#define RF24_REG_SETUP_AW    0x03
#define RF24_REG_SETUP_RETR  0x04
#define RF24_REG_RF_CH       0x05
#define RF24_REG_RF_SETUP    0x06
#define RF24_REG_STATUS      0x07
#define RF24_REG_OBSERVE_TX  0x08
#define RF24_REG_CD          0x09
#define RF24_REG_RPD         0x09
#define RF24_REG_RX_ADDR_P0  0x0A
#define RF24_REG_RX_ADDR_P1  0x0B
#define RF24_REG_RX_ADDR_P2  0x0C
#define RF24_REG_RX_ADDR_P3  0x0D
#define RF24_REG_RX_ADDR_P4  0x0E
#define RF24_REG_RX_ADDR_P5  0x0F
#define RF24_REG_TX_ADDR     0x10
#define RF24_REG_RX_PW_P0    0x11
#define RF24_REG_RX_PW_P1    0x12
#define RF24_REG_RX_PW_P2    0x13
#define RF24_REG_RX_PW_P3    0x14
#define RF24_REG_RX_PW_P4    0x15
#define RF24_REG_RX_PW_P5    0x16
#define RF24_REG_FIFO_STATUS 0x17
#define RF24_REG_DYNPD       0x1C
#define RF24_REG_FEATURE     0x1D

// *** Register Bits *** //
//CONFIG register bitwise definitions
#define RF24_MASK_RX_DR  0x40 // BIT6
#define RF24_MASK_TX_DS  0x20 // BIT5
#define RF24_MASK_MAX_RT 0x10 // BIT4
#define RF24_EN_CRC      0x08 // BIT3
#define RF24_CRC_EN      0x08 // BIT3
#define RF24_CRC_DIS     0x00
#define RF24_CRCO        0x04 // BIT2
#define RF24_CRC_1BYTE   0x00
#define RF24_CRC_2BYTES  0x04
#define RF24_PWR_UP      0x02 // BIT1
#define RF24_PWR_DOWN    0x00 // BIT1
#define RF24_PRIM_RX     0x01 // BIT0
#define RF24_PRIM_TX     0x00 // BIT0

//EN_AA register bitwise definitions
#define RF24_ENAA_P5     0x20 // BIT5
#define RF24_ENAA_P4     0x10 // BIT4
#define RF24_ENAA_P3     0x08 // BIT3
#define RF24_ENAA_P2     0x04 // BIT2
#define RF24_ENAA_P1     0x02 // BIT1
#define RF24_ENAA_P0     0x01 // BIT0

//EN_RXADDR register bitwise definitions
#define RF24_ERX_P5      0x20 // BIT5
#define RF24_ERX_P4      0x10 // BIT4
#define RF24_ERX_P3      0x08 // BIT3
#define RF24_ERX_P2      0x04 // BIT2
#define RF24_ERX_P1      0x02 // BIT1
#define RF24_ERX_P0      0x01 // BIT0

//SETUP_AW register bitwise definitions
#define RF24_AW          0x01 // BIT0

//SETUP_RETR register bitwise definitions
#define RF24_ARD         0x10 // BIT4
#define RF24_ARC         0x01 // BIT0

//RF_CH register bitwise definitions
//#define RF24_CH_CH	 0x01 // BIT0

//RF_SETUP register bitwise definitions
#define RF24_PLL_LOCK    0x10 // BIT4
#define RF24_RF_DR       0x08 // BIT3
#define RF24_RF_PWR      0x02 // BIT1
#define RF24_LNA_HCURR   0x01 // BIT0

//STATUS register bitwise definitions
#define RF24_RX_DR       0x40 // BIT6
#define RF24_TX_DS       0x20 // BIT5
#define RF24_MAX_RT      0x10 // BIT4
#define RF24_RX_P_NO     0x0E // BIT1-3
#define RF24_RX_P_EMPTY  0x0E
#define RF24_TX_FULL     0x01 // BIT0

//OBSERVE_TX register bitwise definitions
#define RF24_PLOS_CNT    0x10 // BIT4
#define RF24_ARC_CNT     0x01 // BIT0

//CD register bitwise definitions
//#define RF24_CD_CD	 0x01 // BIT0

//FIFO_STATUS register bitwise definitions
#define RF24_TX_REUSE    0x40 // BIT6
#define RF24_FIFO_FULL   0x20 // BIT5
#define RF24_TX_EMPTY    0x10 // BIT4
#define RF24_RX_FULL     0x02 // BIT1
#define RF24_RX_EMPTY    0x01 // BIT0

//FEATURE register bitwise definitions
#define RF24_FEATURE_EN_DYNAMIC_PAYLOAD      	0x04 // BIT2
#define RF24_FEATURE_EN_ACK_WITH_PAYLOAD  		0x02 // BIT1
#define RF24_FEATURE_EN_NO_ACK_COMMAND  		0x01 // BIT0

// *** Instructions  *** //
#define RF24_COMMAND_R_REGISTER    		0x00
#define RF24_COMMAND_W_REGISTER    		0x20
#define RF24_COMMAND_R_RX_PAYLOAD  		0x61
#define RF24_COMMAND_W_TX_PAYLOAD  		0xA0
#define RF24_COMMAND_FLUSH_TX      		0xE1
#define RF24_COMMAND_FLUSH_RX      		0xE2
#define RF24_COMMAND_REUSE_TX_PL   		0xE3
#define RF24_COMMAND_R_RX_PL_WID   		0x60
#define RF24_COMMAND_W_ACK_PAYLOAD 		0xA8
#define RF24_COMMAND_W_TX_PAYLOAD_NOACK 0xB0
#define RF24_COMMAND_NOP           		0xFF

/* Available states for the transceiver's state machine */
#define RF24_STATE_NOTPRESENT  0b00000000
#define RF24_STATE_POWERDOWN   0b00000001
#define RF24_STATE_STANDBY_I   0b00000010
#define RF24_STATE_STANDBY_II  0b00000100
#define RF24_STATE_PTX         0b00001000
#define RF24_STATE_PRX         0b00010000
#define RF24_STATE_TEST        0b00100000

/* RF speed settings */
#define RF24_SPEED_1MBPS    0x00
#define RF24_SPEED_2MBPS    0x08
#define RF24_SPEED_250KBPS  0x20
#define RF24_SPEED_MASK     0x08
#define RF24_SPEED_MIN	    RF24_SPEED_250KBPS
#define RF24_SPEED_MAX		RF24_SPEED_2MBPS

/* RF transmit power settings */
#define RF24_POWER_0DBM        0x06
#define RF24_POWER_MINUS6DBM   0x04
#define RF24_POWER_MINUS12DBM  0x02
#define RF24_POWER_MINUS18DBM  0x00
#define RF24_POWER_MAX         RF24_POWER_0DBM
#define RF24_POWER_MIN         RF24_POWER_MINUS18DBM
#define RF24_POWER_MASK        0x06

/* IRQ "reasons" that can be tested. */
#define RF24_IRQ_RX            0x40 // RX is not empty
#define RF24_IRQ_TX            0x20 // TX is empty
#define RF24_IRQ_MAX_RETRANS   0x10 // MAX retransmission count reached
#define RF24_IRQ_MASK          0x70 // RF24_IRQ_RX | RF24_IRQ_TX | RF24_IRQ_MAX_RETRANS

#define RF24_PIPE0 0x00
#define RF24_PIPE1 0x01
#define RF24_PIPE2 0x02
#define RF24_PIPE3 0x03
#define RF24_PIPE4 0x04
#define RF24_PIPE5 0x05

#define RF24_PACKET_SIZE_DYNAMIC     0
#define RF24_PACKET_SIZE_1           1
#define RF24_PACKET_SIZE_2           2
#define RF24_PACKET_SIZE_3           3
#define RF24_PACKET_SIZE_4           4
#define RF24_PACKET_SIZE_5           5
#define RF24_PACKET_SIZE_6           6
#define RF24_PACKET_SIZE_7           7
#define RF24_PACKET_SIZE_8           8
#define RF24_PACKET_SIZE_9           9
#define RF24_PACKET_SIZE_10          10
#define RF24_PACKET_SIZE_11          11
#define RF24_PACKET_SIZE_12          12
#define RF24_PACKET_SIZE_13          13
#define RF24_PACKET_SIZE_14          14
#define RF24_PACKET_SIZE_15          15
#define RF24_PACKET_SIZE_16          16
#define RF24_PACKET_SIZE_17          17
#define RF24_PACKET_SIZE_18          18
#define RF24_PACKET_SIZE_19          19
#define RF24_PACKET_SIZE_20          20
#define RF24_PACKET_SIZE_21          21
#define RF24_PACKET_SIZE_22          22
#define RF24_PACKET_SIZE_23          23
#define RF24_PACKET_SIZE_24          24
#define RF24_PACKET_SIZE_25          25
#define RF24_PACKET_SIZE_26          26
#define RF24_PACKET_SIZE_27          27
#define RF24_PACKET_SIZE_28          28
#define RF24_PACKET_SIZE_29          29
#define RF24_PACKET_SIZE_30          30
#define RF24_PACKET_SIZE_31          31
#define RF24_PACKET_SIZE_32          32

#define RF24_ADRESS_WIDTH_3 0x01
#define RF24_ADRESS_WIDTH_4 0x02
#define RF24_ADRESS_WIDTH_5 0x03

#define RF24_RETRANS_DELAY_250u  0x00
#define RF24_RETRANS_DELAY_500u  0x10
#define RF24_RETRANS_DELAY_750u  0x20
#define RF24_RETRANS_DELAY_1000u 0x30
#define RF24_RETRANS_DELAY_1250u 0x40
#define RF24_RETRANS_DELAY_1500u 0x50
#define RF24_RETRANS_DELAY_1750u 0x60
#define RF24_RETRANS_DELAY_2000u 0x70
#define RF24_RETRANS_DELAY_2250u 0x80
#define RF24_RETRANS_DELAY_2500u 0x90
#define RF24_RETRANS_DELAY_2750u 0xA0
#define RF24_RETRANS_DELAY_3000u 0xB0
#define RF24_RETRANS_DELAY_3250u 0xC0
#define RF24_RETRANS_DELAY_3500u 0xD0
#define RF24_RETRANS_DELAY_3750u 0xE0
#define RF24_RETRANS_DELAY_4000u 0xF0

#define RF24_RETRANS_DISABLE 0x00
#define RF24_RETRANS_COUNT1  0x01
#define RF24_RETRANS_COUNT2  0x02
#define RF24_RETRANS_COUNT3  0x03
#define RF24_RETRANS_COUNT4  0x04
#define RF24_RETRANS_COUNT5  0x05
#define RF24_RETRANS_COUNT6  0x06
#define RF24_RETRANS_COUNT7  0x07
#define RF24_RETRANS_COUNT8  0x08
#define RF24_RETRANS_COUNT9  0x09
#define RF24_RETRANS_COUNT10 0x0A
#define RF24_RETRANS_COUNT11 0x0B
#define RF24_RETRANS_COUNT12 0x0C
#define RF24_RETRANS_COUNT13 0x0D
#define RF24_RETRANS_COUNT14 0x0E
#define RF24_RETRANS_COUNT15 0x0F

#define RF24_BUFFER_SIZE		32
#define RF24_CHECKSUM_SIZE		4
#define RF24_VALID_BUFFER_SIZE	28 // RF24_BUFFER_SIZE - RF24_CHECKSUM_SIZE

#define TXBUFFERSIZE	RF24_BUFFER_SIZE // Size of buffer to store Tx & Rx FIFO data
#define TXPACKETLEN 	TXBUFFERSIZE // Number of Tx bytes to send in a single data transmission

typedef enum
{
	RX_STATUS_SUCCESS,
	RX_STATUS_CRC_ERROR,
	RX_STATUS_INVALID_LENGTH,
	RX_STATUS_FAILED
} e_RxStatus;

// Application methods
bool RfSendPacket(uint8_t *txBuffer);
e_RxStatus RfReceivePacket(uint8_t *rxBuffer);
void initRfModule(bool isEnableInt);
void RfSetChannel(uint8_t chanNum);
void RfSetRxMode(void);
void RfFlushTxFifo(void);
void RfFlushRxFifo(void);

void RfWaitUs(uint32_t periodUs);

bool RfTryToGetRxPacket(uint64_t ui64PeriodInUs,
			bool (*pfnDecodePacket)(uint8_t* pRxBuff, va_list argp), ...);

bool RfTryToCaptureRfSignal(uint64_t ui64PeriodInUs,
			bool (*pfnHandler)(va_list argp), ...);

//-----------------------FUNCTIONS----------------------------//
// Initialization and configuration
void RF24_init(const RF24_InitTypeDef* InitRf24);

// Test if chip is present and/or SPI is working.
uint8_t RF24_isAlive ();

// Configure RF configure register
void RF24_setConfigureRegister (uint8_t newConfigure);

// Read RF configure register
uint8_t RF24_getConfigureRegister ();

// Commit RF speed & TX power from rf_speed_power variable.
void RF24_setSpeedAndPower (uint8_t rfSpeedAndPower);

// Commit Enhanced ShockBurst Address Width from rf_addr_width variable.
void RF24_setAddressWidth (uint8_t rfAddressWidth);

// Enable specified feature (RF24_EN_* from nRF24L01.h, except RF24_EN_CRC)
void RF24_enableFeatures (uint8_t feature);

// Disable specified feature
void RF24_disableFeatures (uint8_t feature);

// Get current state of the nRF24L01+ chip, test with RF24_STATE_* #define's
uint8_t RF24_getCurrentState ();

// Number of packets lost since last time the Channel was set.
uint8_t RF24_getLostPackets ();

// Check if the IRQ flag is set or not
uint8_t RF24_getIrqFlag (uint8_t rfIrqFlag);

// Clear specified Interrupt Flags
void RF24_clearIrqFlag (uint8_t irqflag);

// Enter Power-Down mode (0.9uA power draw)
void RF24_startPowerDown ();

// Enter Standby-I mode (26uA power draw)
void RF24_startStandby ();

// Scan current channel for RPD (looks for any signals > -64dBm)
uint8_t RF24_scan ();

// Send read register command and read data of the corresponding register
uint8_t RF24_readRegister	(uint8_t addr);

// Send write register command and write data to the corresponding register
void RF24_writeRegister	(uint8_t addr, uint8_t data);

// Configure TX address to send next packet
void RF24_TX_setAddress (uint8_t *addr);

// Change the RF board to TX state and keep it at the STANDBY-I mode
void RF24_TX_activate ();

// TX FIFO contents will be sent over the air (~320uA STBY2, 7-11mA PTX)
// Then it is put int the STANDY-I mode again
void RF24_TX_pulseTransmit();

// Send data to other RF boards and request ack (if enable)
void RF24_TX_writePayloadAck (uint8_t len, uint8_t *data);

// Send data to other RF boards and not request ack (feature must be enabled in init function)
void RF24_TX_writePayloadNoAck (uint8_t len, uint8_t *data);

//// Flush every pending payload in the TX FIFO
//void RF24_TX_flush ();

//Enable retransmitting contents of TX FIFO endlessly until RF24_flush_tx() or the FIFO contents are replaced.
void RF24_TX_reuseLastPayload ();

// Configure RX address of "rf_addr_width" size into the specified pipe
void RF24_RX_setAddress (uint8_t pipe, uint8_t *addr);

// Get the size of incoming RX payload; Only available in dynamic payload mode
uint8_t RF24_RX_getPayloadWidth ();

// Read the received data
uint8_t RF24_RX_getPayloadData (uint8_t len, uint8_t *data);

//// Enable PRX mode (~12-14mA power draw)
//void RF24_RX_activate ();

//// Flush every peding payload in the RX FIFO
//void RF24_RX_flush ();

// Check if RX FIFO data is available for reading.
uint8_t RF24_RX_isEmpty ();

// Used to manually ACK with a payload.  Must have RF24_EN_ACK_PAY enabled; this is not enabled by default
// When RF24_EN_ACK_PAY is enabled on the PTX side, ALL transmissions must be manually ACK'd by the receiver this way.
// The receiver (PRX) side needs to have RF24_EN_ACK_PAY enabled too, or else it will automatically ACK with a zero-byte packet.
//
// If you have this enabled on the PTX but not the PRX, the transmission will go through and the PRX will receive/notify about
// the RX payload, but the PTX will ignore the zero-byte autoack from the PRX and perform its retransmit sequence, erroring
// out with MAX_RT (RF24_IRQ_TXFAILED) after (RF24_SETUP_RETR >> RF24_ARC) retransmissions.
// When this occurs, the PRX will still only notify its microcontroller of the payload once (the PID field in the packet uniquely
// identifies it so the PRX knows it's the same packet being retransmitted) but it's obviously wasting on-air time (and power).
void RF24_RX_sendAckWithPayload (uint8_t pipe, uint8_t len, uint8_t *data);

// Detect Carrier Signal in the configured channel
bool RF24_RX_carrierDetection();

// Disable specified RX pipe
void RF24_PIPE_close (uint8_t pipe);

// Disable all RX pipes (used during initialization)
void RF24_PIPE_closeAll ();

// Enable specified RX pipe, optionally turn auto-ack (Enhanced ShockBurst) on
void RF24_PIPE_open (uint8_t pipe, bool autoack);

// Check if specified RX pipe is active
uint8_t RF24_PIPE_isOpen (uint8_t pipe);

// Set static length of pipe's RX payloads (1-32), size=0 enables DynPD.
void RF24_PIPE_setPacketSize (uint8_t pipe, uint8_t size);

// 500-4000uS range, clamped by RF speed
void RF24_RETRANS_setDelay (uint8_t delay);

// 0-15 retransmits before MAX_RT (RF24_IRQ_TXFAILED) IRQ raised
void RF24_RETRANS_setCount (uint8_t count);

// Number of times a packet was retransmitted during last TX attempt
uint8_t RF24_RETRANS_getLastRetransmits ();

//this thing is too long so I left it here
#define RF24_CHANNEL_0          0
#define RF24_CHANNEL_1          1
#define RF24_CHANNEL_2          2
#define RF24_CHANNEL_3          3
#define RF24_CHANNEL_4          4
#define RF24_CHANNEL_5          5
#define RF24_CHANNEL_6          6
#define RF24_CHANNEL_7          7
#define RF24_CHANNEL_8          8
#define RF24_CHANNEL_9          9
#define RF24_CHANNEL_10          10
#define RF24_CHANNEL_11          11
#define RF24_CHANNEL_12          12
#define RF24_CHANNEL_13          13
#define RF24_CHANNEL_14          14
#define RF24_CHANNEL_15          15
#define RF24_CHANNEL_16          16
#define RF24_CHANNEL_17          17
#define RF24_CHANNEL_18          18
#define RF24_CHANNEL_19          19
#define RF24_CHANNEL_20          20
#define RF24_CHANNEL_21          21
#define RF24_CHANNEL_22          22
#define RF24_CHANNEL_23          23
#define RF24_CHANNEL_24          24
#define RF24_CHANNEL_25          25
#define RF24_CHANNEL_26          26
#define RF24_CHANNEL_27          27
#define RF24_CHANNEL_28          28
#define RF24_CHANNEL_29          29
#define RF24_CHANNEL_30          30
#define RF24_CHANNEL_31          31
#define RF24_CHANNEL_32          32
#define RF24_CHANNEL_33          33
#define RF24_CHANNEL_34          34
#define RF24_CHANNEL_35          35
#define RF24_CHANNEL_36          36
#define RF24_CHANNEL_37          37
#define RF24_CHANNEL_38          38
#define RF24_CHANNEL_39          39
#define RF24_CHANNEL_40          40
#define RF24_CHANNEL_41          41
#define RF24_CHANNEL_42          42
#define RF24_CHANNEL_43          43
#define RF24_CHANNEL_44          44
#define RF24_CHANNEL_45          45
#define RF24_CHANNEL_46          46
#define RF24_CHANNEL_47          47
#define RF24_CHANNEL_48          48
#define RF24_CHANNEL_49          49
#define RF24_CHANNEL_50          50
#define RF24_CHANNEL_51          51
#define RF24_CHANNEL_52          52
#define RF24_CHANNEL_53          53
#define RF24_CHANNEL_54          54
#define RF24_CHANNEL_55          55
#define RF24_CHANNEL_56          56
#define RF24_CHANNEL_57          57
#define RF24_CHANNEL_58          58
#define RF24_CHANNEL_59          59
#define RF24_CHANNEL_60          60
#define RF24_CHANNEL_61          61
#define RF24_CHANNEL_62          62
#define RF24_CHANNEL_63          63
#define RF24_CHANNEL_64          64
#define RF24_CHANNEL_65          65
#define RF24_CHANNEL_66          66
#define RF24_CHANNEL_67          67
#define RF24_CHANNEL_68          68
#define RF24_CHANNEL_69          69
#define RF24_CHANNEL_70          70
#define RF24_CHANNEL_71          71
#define RF24_CHANNEL_72          72
#define RF24_CHANNEL_73          73
#define RF24_CHANNEL_74          74
#define RF24_CHANNEL_75          75
#define RF24_CHANNEL_76          76
#define RF24_CHANNEL_77          77
#define RF24_CHANNEL_78          78
#define RF24_CHANNEL_79          79
#define RF24_CHANNEL_80          80
#define RF24_CHANNEL_81          81
#define RF24_CHANNEL_82          82
#define RF24_CHANNEL_83          83
#define RF24_CHANNEL_84          84
#define RF24_CHANNEL_85          85
#define RF24_CHANNEL_86          86
#define RF24_CHANNEL_87          87
#define RF24_CHANNEL_88          88
#define RF24_CHANNEL_89          89
#define RF24_CHANNEL_90          90
#define RF24_CHANNEL_91          91
#define RF24_CHANNEL_92          92
#define RF24_CHANNEL_93          93
#define RF24_CHANNEL_94          94
#define RF24_CHANNEL_95          95
#define RF24_CHANNEL_96          96
#define RF24_CHANNEL_97          97
#define RF24_CHANNEL_98          98
#define RF24_CHANNEL_99          99
#define RF24_CHANNEL_100         100
#define RF24_CHANNEL_101         101
#define RF24_CHANNEL_102         102
#define RF24_CHANNEL_103         103
#define RF24_CHANNEL_104         104
#define RF24_CHANNEL_105         105
#define RF24_CHANNEL_106         106
#define RF24_CHANNEL_107         107
#define RF24_CHANNEL_108         108
#define RF24_CHANNEL_109         109
#define RF24_CHANNEL_110         110
#define RF24_CHANNEL_111         111
#define RF24_CHANNEL_112         112
#define RF24_CHANNEL_113         113
#define RF24_CHANNEL_114         114
#define RF24_CHANNEL_115         115
#define RF24_CHANNEL_116         116
#define RF24_CHANNEL_117         117
#define RF24_CHANNEL_118         118
#define RF24_CHANNEL_119         119
#define RF24_CHANNEL_120         120
#define RF24_CHANNEL_121         121
#define RF24_CHANNEL_122         122
#define RF24_CHANNEL_123         123
#define RF24_CHANNEL_124         124
#define RF24_CHANNEL_125         125

#ifdef __cplusplus
}
#endif

#endif //_NRF24L01_H

#endif
