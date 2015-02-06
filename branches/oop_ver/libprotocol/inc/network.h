/*
 * network.h
 *
 *  Created on: Jan 4, 2015
 *      Author: VyLong
 */

#ifndef NETWORK_H_
#define NETWORK_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

#ifdef RF_USE_CC2500
#include "libcc2500/inc/cc2500.h"
#endif

#ifdef RF_USE_nRF24L01
#include "libnrf24l01/inc/nRF24L01.h"
#endif

//===============================================================
//  Header index definition
//  only for manual construct with bytes
//  Not recommended to USED!!!
//
//  Example: To construct a header which have
//	+Length = 11
//	+Status_PID = 5
//	+Status_PacketType = PACKET_NACK
//	+Status_HandshakeType = HANDSHAKE_ACK
//	+Checksum = 0x5678
//	+Source Adderss = 0x00BD5A04
//	+Destiantion Adderss = 0x00BD5A03
//  	Implement code:
//			uint8_t buffer[12] = {0};
//
//  		buffer[RF_HEADER_LEN] = 11;
//
//			buffer[RF_HEADER_STATUS] = 0x25;
//
//			buffer[RF_HEADER_CHECKSUM_LOW] = 0x78;
//			buffer[RF_HEADER_CHECKSUM_HIGH] = 0x56;
//
//			buffer[RF_HEADER_SRCADDR_LOW] = 0x04;
//			buffer[RF_HEADER_SRCADDR_UPPERLOW] = 0x5A;
//			buffer[RF_HEADER_SRCADDR_HIGH] = 0xBD;
//			buffer[RF_HEADER_SRCADDR_UPPERHIGH] = 0x00;
//
//			buffer[RF_HEADER_DESADDR_LOW] = 0x03;
//			buffer[RF_HEADER_DESADDR_UPPERLOW] = 0x5A;
//			buffer[RF_HEADER_DESADDR_HIGH] = 0xBD;
//			buffer[RF_HEADER_DESADDR_UPPERHIGH] = 0x00;
//
//			<HeaderPointer> = (Network::Header*)buffer;
//===============================================================
#define RF_HEADER_LEN					0
//#define RF_HEADER_STATUS				1	// (RF_HEADER_LEN + 1)
#define RF_HEADER_CHECKSUM_LOW			2	// (RF_HEADER_STATUS + 1)
#define RF_HEADER_CHECKSUM_HIGH			3	// (RF_HEADER_CHECKSUM_LOW + 1)
//#define RF_HEADER_SRCADDR_LOW			4	// (RF_HEADER_CHECKSUM_HIGH + 1)
//#define RF_HEADER_SRCADDR_UPPERLOW		5	// (RF_HEADER_SRCADDR_LOW + 1)
//#define RF_HEADER_SRCADDR_HIGH			6	// (RF_HEADER_SRCADDR_UPPERLOW + 1)
//#define RF_HEADER_SRCADDR_UPPERHIGH		7	// (RF_HEADER_SRCADDR_HIGH + 1)
//#define RF_HEADER_DESADDR_LOW			8	// (RF_HEADER_SRCADDR_UPPERHIGH + 1)
//#define RF_HEADER_DESADDR_UPPERLOW		9	// (RF_HEADER_DESADDR_LOW + 1)
//#define RF_HEADER_DESADDR_HIGH			10	// (RF_HEADER_DESADDR_UPPERLOW + 1)
//#define RF_HEADER_DESADDR_UPPERHIGH		11	// (RF_HEADER_DESADDR_HIGH + 1)

#define RF_PID_MASK		0x0F

#define RF_HEADER_LENGTH		12	// This is the sizeof(Header)
#define RF_DATA_LENGTH			16	// Spaces for application message
#define RF_PACKET_LENGTH		28	// Total packet length (RF_HEADER_LENGTH + RF_DATA_LENGTH)
#define RF_MESSAGE_SIZE_LENGTH	4	// Spaces for application message size indicator, only exist in Single or First packet
#define RF_DATA_START_INDEX		RF_HEADER_LENGTH	// The begining of data field in rf packet

#define RF_CHECKSUM_PREOFFSET	0x0000	// This value, should be zero, is add to the sum result before generate checksum

//*****************************************************************************
// Convert 4 bytes of an array to a 32-bit value
// @param x: The converted byte array
// @return : a 32-bit value.
//*****************************************************************************
#define construct4Byte(x)	((*x << 24) | (*(x+1) << 16) |	\
							(*(x+2) << 8) | *(x+3))


#define parse32bitTo4Bytes(x, y)	{*(x) = y >> 24; \
									 *(x + 1) = y >> 16; \
									 *(x + 2) = y >> 8; \
									 *(x + 3) = y;}

#define PACKET_LIFETIME				15		// Packet retransmit times for Tx failed...
#define ACK_TIMEOUT_USEC			1500	// Waiting for ACK response period in mircosecond
#define	RETRANSMIT_DELAY_USEC		2000	// The period delay before retransmit packet in mircosecond
#define ACK_PACKET_DELAY_USEC		500		// The period delay before transmit ACK in mircosecond
#define ACK_PACKET_LENGTH			RF_HEADER_LENGTH	// Ack response packet length, this packet not contain data field
#define MULTIPACKET_TIMEOUT_USEC	10000	// Waiting for Middle/Last packet period in mircosecond

#define NETWORK_BROADCAST_ADDRESS	0x00BEADFF

typedef enum tag_NetworkState
{
	STATE_IDLE,
	STATE_WAIT_FOR_DATA,
	STATE_DATA_AVAILABLE
}e_NetworkState;

typedef enum tag_HandShake
{
	HANDSHAKE_ACK = 0,
	HANDSHAKE_NOACK = 1
} e_HandShake;

typedef enum tag_Packet
{
	PACKET_HELLO = 0,
	PACKET_ACK = 1,
	PACKET_NACK = 2,
	PACKET_SINGLE = 3,
	PACKET_FIRST = 4,
	PACKET_MIDDLE = 5,
	PACKET_LAST = 6
} e_Packet;

typedef enum tag_HandShakeReturn
{
	HANDSHAKERETURN_INVALID,
	HANDSHAKERETURN_DUPLICATED,
	HANDSHAKERETURN_SUCCESS,
	HANDSHAKERETURN_HELLO
} e_HandShakeReturn;

typedef struct tag_Status {
	// WARNING!!! Do not change this order unless you know what you are doing!
	// Changing this order due to wrong RF_PID_MASK definition at top file
	unsigned char ui4Pid : 4;
	e_Packet	ePacketType : 3;
	e_HandShake eHandShakeType : 1;
} Status;

typedef struct tag_Header {
	// WARNING!!! Do not change this order unless you know what you are doing!
	// Changing this order due to wrong RF_HEADER_ definition at top file
	uint8_t  ui8Length;
	Status   status;
	uint16_t ui16Checksum;
	uint32_t ui32SourceAddress;
	uint32_t ui32DestinationAddress;
} Header;


void Network_changeState(e_NetworkState state);
e_NetworkState Network_getState();

void Network_setSelfAddress(uint32_t ui32Addr);
uint32_t Network_getSelfAddress();

uint8_t Network_generateNextPID(uint8_t ui8BasePID);
void Network_setLastTxPID(uint8_t ui8PID);
uint8_t Network_getLastTxPID();

void Network_setLastRxPID(uint8_t ui8PID);
uint8_t Network_getLastRxPID();

void Network_setLastRxChecksum(uint16_t ui16Checksum);
uint16_t Network_getLastRxChecksum();

bool Network_sendMessage(uint32_t ui32DestAddr, uint8_t *pui8Message,
		uint32_t ui32MessSize, bool bIsAckRequire);
bool Network_isAckPacket(uint8_t* pRxBuff, va_list argp);
bool Network_receivedMessage(uint8_t** ppui8MessBuffer, uint32_t* pui32MessSize);
e_HandShakeReturn Network_isHandShakeProcessSuccess(Header** ppRxHeader, uint8_t* pui8RxBuffer);
bool Network_isNextPacket(uint8_t* pRxBuff, va_list argp);

void Network_sendACK(Header RxHeader);

void Network_deleteBuffer(uint8_t *pui8Buff);

#ifdef __cplusplus
}
#endif

#endif /* NETWORK_H_ */
