/*
 * network.cpp
 *
 *  Created on: Jan 4, 2015
 *      Author: VyLong
 */

#ifndef NETWORK_CPP_
#define NETWORK_CPP_

#include "libprotocol/inc/network.h"

static uint32_t g_ui32SelfAddress = 0;
static uint8_t g_ui8LastTxPID = 0;
static e_NetworkState g_eCurrentState = STATE_IDLE;

static uint8_t g_ui8LastRxPID = 0;
static uint16_t g_ui16LastRxChecksum = 0x0000;

//-----------------------------------------------------------------------------
//  Network::Network()
//
//  DESCRIPTION:
//  Network protocol constructor
//-----------------------------------------------------------------------------
//Network::Network()
//{
//	ui32SelfAddress = 0x00BD5A01;	//TODO: load EEPROM value instead of hardcode
//	ui8LastUsedPID = 0;				//TODO: generate random PID instead of hardcode
//	eCurrentState = Network::STATE_IDLE;
//}

//-----------------------------------------------------------------------------
//  void Network::changeState(Network::e_NetworkState newState)
//
//  DESCRIPTION:
//  This function set current network state to newState.
//
//  ARGUMENTS:
//      Network::e_NetworkState newState
//          New network protocol state
//-----------------------------------------------------------------------------
void Network_changeState(e_NetworkState newState)
{
	g_eCurrentState = newState;
}

//-----------------------------------------------------------------------------
//  Network::e_NetworkState Network::getState()
//
//  DESCRIPTION:
//  This function return current network state.
//
//  RETURN VALUE:
//      Network::e_NetworkState
//			STATE_IDLE				: The protocol is free
//			STATE_WAIT_FOR_DATA 	: The protocol is waitting for new packet
//			STATE_DATA_AVAILABLE 	: The protocol is fully received a message.
//-----------------------------------------------------------------------------
e_NetworkState Network_getState()
{
	return g_eCurrentState;
}

//-----------------------------------------------------------------------------
//  void Network::setSelfAddress(uint32_t ui32Addr)
//
//  DESCRIPTION:
//  This function set current network address
//
//  ARGUMENTS:
//      uint32_t ui32Addr
//          New network address
//-----------------------------------------------------------------------------
void Network_setSelfAddress(uint32_t ui32Addr)
{
	g_ui32SelfAddress = ui32Addr;
}

//-----------------------------------------------------------------------------
//  uint32_t Network::getSelfAddress()
//
//  DESCRIPTION:
//  This function return current network address.
//
//  RETURN VALUE:
//      uint32_t
//			Current self address
//-----------------------------------------------------------------------------
uint32_t Network_getSelfAddress()
{
	return g_ui32SelfAddress;
}


//-----------------------------------------------------------------------------
//  uint8_t Network::generateNextPID(uint8_t ui8BasePID)
//
//  DESCRIPTION:
//  This function return the next PID base on ui8BasePID
//
//  ARGUMENTS:
//		uint8_t ui8BasePID
//			Base number use to generate new PID number
//
//  RETURN VALUE:
//      uint8_t
//			Packet Identification (PID) number
//-----------------------------------------------------------------------------
uint8_t Network_generateNextPID(uint8_t ui8BasePID)
{
	return ((ui8BasePID + 1) & RF_PID_MASK);	// Can be modify to new PID generate algorithm
}

//-----------------------------------------------------------------------------
//  uint8_t Network::setLastTxPID(uint8_t ui8PID)
//
//  DESCRIPTION:
//  This function set the last PID to ui8PID
//
//  ARGUMENTS:
//      uint8_t
//			PID number need to be stored
//-----------------------------------------------------------------------------
void Network_setLastTxPID(uint8_t ui8PID)
{
	g_ui8LastTxPID = ui8PID;
}

//-----------------------------------------------------------------------------
//  uint8_t Network::getLastTxPID()
//
//  DESCRIPTION:
//  This function return the previous PID
//
//  RETURN VALUE:
//      uint8_t
//			Last used Packet Identification (PID) number
//-----------------------------------------------------------------------------
uint8_t Network_getLastTxPID()
{
	return g_ui8LastTxPID;
}

//-----------------------------------------------------------------------------
//  uint8_t Network::setLastRxPID(uint8_t ui8PID)
//
//  DESCRIPTION:
//  This function save the last received PID to ui8PID
//
//  ARGUMENTS:
//      uint8_t
//			PID number need to be stored
//-----------------------------------------------------------------------------
void Network_setLastRxPID(uint8_t ui8PID)
{
	g_ui8LastRxPID = ui8PID;
}

//-----------------------------------------------------------------------------
//  uint8_t Network::getLastRxPID()
//
//  DESCRIPTION:
//  This function return the previous received packet PID
//
//  RETURN VALUE:
//      uint8_t
//			Last received Packet Identification (PID) number
//-----------------------------------------------------------------------------
uint8_t Network_getLastRxPID()
{
	return g_ui8LastRxPID;
}

//-----------------------------------------------------------------------------
//  uint8_t Network::getLastRxChecksum(uint16_t ui16Checksum)
//
//  DESCRIPTION:
//  This function save the last received packet checksum to ui16Checksum
//
//  ARGUMENTS:
//      uint16_t ui16Checksum
//			Checksum number need to be stored
//-----------------------------------------------------------------------------
void Network_setLastRxChecksum(uint16_t ui16Checksum)
{
	g_ui16LastRxChecksum = ui16Checksum;
}

//-----------------------------------------------------------------------------
//  uint8_t Network::getLastRxChecksum()
//
//  DESCRIPTION:
//  This function return the previous received packet checksum
//
//  RETURN VALUE:
//      uint8_t
//			Last received Packet checksum number
//-----------------------------------------------------------------------------
uint16_t Network_getLastRxChecksum()
{
	return g_ui16LastRxChecksum;
}

//-----------------------------------------------------------------------------
//  bool Network::sendMessage(uint32_t ui32DestAddr, uint8_t *pui8Message,
//				uint32_t ui32MessSize, bool bIsAckRequire)
//
//  DESCRIPTION:
//  This function transmit a message to specific destination address.
//  The first 4 byte in the message buffer must hold the message size.
//  This is valid message format for the transmission.
//
//	WARING! Must disable IRQ interrupt feature before call this function
//			and only re-enable IRQ interrupt after this function return if need
//
//  ARGUMENTS:
//		uint32_t ui32DestAddr
//			Message destination address.
//		uint8_t *pui8Message
//			Message buffer containt message size and data.
//		uint32_t ui32MessSize
//			Message size
//		bool bIsAckRequire
//			If true mean this transmission is reliable because there are ACK responsable
//			otherwise broadcast transmission is known.
//
//  RETURN VALUE:
//      bool
//			true	: Transmission successful!
//			false 	: Transmission failed...
//-----------------------------------------------------------------------------
bool Network_sendMessage(uint32_t ui32DestAddr, uint8_t *pui8Message,
			uint32_t ui32MessSize, bool bIsAckRequire)
{
	uint32_t i;

	// Transmission manipulation variables
	bool bIsTxSuccess;
	int8_t i8TxPacketLifeTime;

	// Message manipulation variables
	uint8_t ui8PacketSize;
	uint32_t ui32MessagePointer = 0;
	int32_t i32RemainMessSize = ui32MessSize;
	uint16_t ui16Checksum;

	// Create packet holder
	uint8_t pui8TxBuffer[RF_PACKET_LENGTH] = {0};

	// Construct header
	Header header;
	Header* pHeader;
	pHeader = &header;

	header.ui32SourceAddress = Network_getSelfAddress();
	header.ui32DestinationAddress = ui32DestAddr;
	if (ui32MessSize > RF_DATA_LENGTH)
	{
		header.ui8Length = (RF_PACKET_LENGTH - 1);
		header.status.ePacketType = PACKET_FIRST;
	}
	else
	{
		header.ui8Length = ui32MessSize + (RF_HEADER_LENGTH - 1);
		header.status.ePacketType = PACKET_SINGLE;
	}
	Network_setLastTxPID(Network_generateNextPID(Network_getLastTxPID()));
	header.status.ui4Pid = Network_getLastTxPID();

	header.status.eHandShakeType = (bIsAckRequire) ?
			(HANDSHAKE_ACK) : (HANDSHAKE_NOACK);
	header.ui16Checksum = RF_CHECKSUM_PREOFFSET;

	//-- loop here
	while(true)
	{
		// Fill header, checksum is not correct at here
		for(i = 0; i < RF_HEADER_LENGTH; i++)
		{
			pui8TxBuffer[i] = *((uint8_t*)(pHeader) + i);
		}

		// Fill message
		if(header.status.ePacketType == PACKET_FIRST)
		{
			// Fill message size only for Sigle/First packet
			parse32bitTo4Bytes(&pui8TxBuffer[RF_HEADER_LENGTH], ui32MessSize);

			ui8PacketSize = RF_HEADER_LENGTH + RF_MESSAGE_SIZE_LENGTH;
			for(i = 0; i < i32RemainMessSize && ui8PacketSize < RF_PACKET_LENGTH; i++)
			{
				pui8TxBuffer[i + RF_HEADER_LENGTH + RF_MESSAGE_SIZE_LENGTH] = pui8Message[ui32MessagePointer++];
				ui8PacketSize++;
			}
		}
		else // Single, middle, last packet
		{
			ui8PacketSize = RF_HEADER_LENGTH;
			for(i = 0; i < i32RemainMessSize && ui8PacketSize < RF_PACKET_LENGTH; i++)
			{
				pui8TxBuffer[i + RF_HEADER_LENGTH] = pui8Message[ui32MessagePointer++];
				ui8PacketSize++;
			}
		}
		i32RemainMessSize -= i;
		i32RemainMessSize = (i32RemainMessSize < 0) ? (0) : (i32RemainMessSize);

		// Generate checksum
		ui16Checksum = 0;
		for(i = 0; i < ui8PacketSize; i++)
			ui16Checksum += pui8TxBuffer[i];
		header.ui16Checksum = ~(ui16Checksum) + 1;

		// Refill checksum at header
		pui8TxBuffer[RF_HEADER_CHECKSUM_LOW] = (uint8_t)(header.ui16Checksum);
		pui8TxBuffer[RF_HEADER_CHECKSUM_HIGH] = (uint8_t)(header.ui16Checksum >> 8);

		// Restransmission process start here
		bIsTxSuccess = false;
		i8TxPacketLifeTime = PACKET_LIFETIME;
		do{
			// Send packet
			if(RfSendPacket(pui8TxBuffer))
			{
				// Monitoring ACK response
				if(!bIsAckRequire)
				{
					bIsTxSuccess = true;
					break;
				}
				else
				{
					// Is ACK packet received?
					if(RfTryToGetRxPacket(ACK_TIMEOUT_USEC, Network_isAckPacket, &header) == true)
					{
						bIsTxSuccess = true;	// Packet transmit success
						break;
					}
					else	// Transmission failed!
					{
						// Decrease packet life time
						i8TxPacketLifeTime--;

						// Immedialy check for fast return without delay after the last try
						if(i8TxPacketLifeTime > 0)
						{
							// Delay before retransmit
							RfWaitUs(RETRANSMIT_DELAY_USEC);
							continue;
						}
						else
						{
							break;
						}
					}
				}
			}
			else
			{
				// Something WRONG!!! This case only happen at runtime error (Tx-if-CCA failed)
				// or wrong data format (TxBuffer[0] equal zero)
				i8TxPacketLifeTime--;
			}
		} while(i8TxPacketLifeTime > 0); // Can be replace which: 'while(true)'

		// Final process
		if (bIsTxSuccess)
		{
			// Is finish transmission?
			if (i32RemainMessSize <= 0)
			{
				return true;	// Transmission successful!
			}
			else
			{
				// Modify next packet header
				if (i32RemainMessSize > RF_DATA_LENGTH)
				{
					header.ui8Length = RF_PACKET_LENGTH - 1;
					header.status.ePacketType = PACKET_MIDDLE;
				}
				else
				{
					header.ui8Length = i32RemainMessSize + (RF_HEADER_LENGTH - 1);
					header.status.ePacketType = PACKET_LAST;
				}
				Network_setLastTxPID(Network_generateNextPID(Network_getLastTxPID()));
				header.status.ui4Pid = Network_getLastTxPID();
				header.ui16Checksum = RF_CHECKSUM_PREOFFSET;
				continue;
			}
		}
		//else
		return false; // Transmission failed...
	}
}

//-----------------------------------------------------------------------------
//  bool Network::isAckPacket(uint8_t* pRxBuff, va_list argp)
//
//  DESCRIPTION:
//  This function spin until ACK packet received or ACKTIMEOUTUSEC exprised
//
//  ARGUMENTS:
//		uint8_t* pRxBuff
//			This pointer point to the received packet
//		va_list argp
//			This list containt one argument:
//			1/ 	Header* pTxHeader
//				Point to the tx packet header which require ACK
//
//  RETURN VALUE:
//      bool
//			true	: Recieved ACK
//			false 	: Non-ACK packet received
//-----------------------------------------------------------------------------
bool Network_isAckPacket(uint8_t* pRxBuff, va_list argp)
{
	// Get the input arguments
	Header* pTxHeader;
	pTxHeader = va_arg(argp, Header*);

	// Rewap received packet
	Header* pRxHeader;
	pRxHeader = (Header*)(pRxBuff);

	// Is correct header?
	if (pRxHeader->ui8Length == (ACK_PACKET_LENGTH - 1)
			&& pRxHeader->ui32DestinationAddress == pTxHeader->ui32SourceAddress
			&& pRxHeader->ui32SourceAddress == pTxHeader->ui32DestinationAddress
			&& pRxHeader->status.ui4Pid == Network_generateNextPID(pTxHeader->status.ui4Pid)
			&& pRxHeader->status.ePacketType == PACKET_ACK)
	{
		return true;	// Received ACK
	}
	return false;	// Non-ACK packet
}

//-----------------------------------------------------------------------------
//  bool Network::receivedMessage(uint8_t** ppui8MessBuffer, uint32_t* pui32MessSize)
//
//  DESCRIPTION:
//  This function try to construct a recieved message from transmitter
//  The first 4 byte in the message buffer must hold the message size.
//  This is valid message format for the transmission.
//
//  WARNING!!! Only use this function in interrupt handler
//
//  ARGUMENTS:
//		uint8_t** ppui8MessBuffer
//			Pointer point to the pointer point to the return message storage
//		uint32_t* pui32MessSize
//			Point to a 32-bit values which indicate the message size
//
//  RETURN VALUE:
//      bool
//			true	: Construct completed  a message!
//			false 	: This transaction is terminal, discard message!
//-----------------------------------------------------------------------------
bool Network_receivedMessage(uint8_t** ppui8MessBuffer, uint32_t* pui32MessSize)
{
	uint32_t i;

	// Packet manipulation variables
	Header* pRxHeader = 0;
	e_RxStatus eRxStatus;
	uint8_t ui8DataStartIndex;

	// Message manipulation variables
	uint32_t ui32MessSize;
	uint32_t ui32ReceivedSize;
	uint8_t ui8RxMessSize = 0;
	uint32_t ui32MessPointer = 0;

	// Create packet holder
	uint8_t pui8RxBuffer[RF_PACKET_LENGTH] = {0};

	// Fetch packet from CCxxxx
	eRxStatus = RfReceivePacket(pui8RxBuffer);

	switch(eRxStatus)
	{
	case RX_STATUS_SUCCESS:
		if(Network_isHandShakeProcessSuccess(&pRxHeader, pui8RxBuffer) == HANDSHAKERETURN_SUCCESS)
		{
			if(pRxHeader->status.ePacketType == PACKET_SINGLE || pRxHeader->status.ePacketType == PACKET_FIRST)
			{
				// Get message size
				if(pRxHeader->status.ePacketType == PACKET_FIRST)
				{
					ui32MessSize = construct4Byte(&pui8RxBuffer[RF_DATA_START_INDEX]);
					ui32ReceivedSize = pRxHeader->ui8Length - (RF_HEADER_LENGTH - 1) - RF_MESSAGE_SIZE_LENGTH;

					ui8DataStartIndex = RF_MESSAGE_SIZE_LENGTH + RF_DATA_START_INDEX;
				}
				else // Single
				{
					ui32MessSize = pui8RxBuffer[RF_HEADER_LEN] - (RF_HEADER_LENGTH - 1);
					ui32ReceivedSize = pRxHeader->ui8Length - (RF_HEADER_LENGTH - 1);

					ui8DataStartIndex = RF_DATA_START_INDEX;
				}

				// Allocated message buffer space
				(*ppui8MessBuffer) = new uint8_t[ui32MessSize];
				if((*ppui8MessBuffer) == 0)
					return false;

				// Fill message content
				for(i = 0; i < ui32ReceivedSize; i++)
				{
					(*ppui8MessBuffer)[ui32MessPointer++] = pui8RxBuffer[i + ui8DataStartIndex];
				}

				// Signel or multi packet process
				switch(pRxHeader->status.ePacketType)
				{
				case PACKET_SINGLE:
					// Check for invalid message length
					if(ui32ReceivedSize == ui32MessSize)
					{
						*pui32MessSize = ui32ReceivedSize;
						return true; // Completed received a message!
					}
					break;

				case PACKET_FIRST:
					//--loop here
					do
					{
						if(RfTryToGetRxPacket(MULTIPACKET_TIMEOUT_USEC, Network_isNextPacket, Network_getLastRxPID(), pui8RxBuffer, &ui8RxMessSize) == true)
						{
							// Fill message
							for(i = 0; i < ui8RxMessSize; i++)
							{
								(*ppui8MessBuffer)[ui32MessPointer++] = pui8RxBuffer[i + RF_HEADER_LENGTH];
							}
							ui32ReceivedSize += ui8RxMessSize;
						}
						else
							break;	// Transaction terminal!
					} while(ui32ReceivedSize < ui32MessSize);

					if(ui32ReceivedSize == ui32MessSize)
					{
						*pui32MessSize = ui32ReceivedSize;
						return true; // Completed received a message!
					}
					break;

				default:
					break;
				}
			}
		}
		break;

	case RX_STATUS_CRC_ERROR:
		//TODO: generate JAMMING signal in few milisecond !!!
		break;

	default:	// RX_STATUS_FAILED
		break;
	}

	*pui32MessSize = 0;
	return false;
}

//-----------------------------------------------------------------------------
//  e_HandShakeReturn Network::isHandShakeProcessSuccess(Header** ppRxHeader, uint8_t* pui8RxBuffer)
//
//  DESCRIPTION:
//  This function perform sequence check with the received packet.
//	If the destination address not equal this network address and
//	the packet not is broadcast type then return. Orthewise, if ack
//	require then transmit ACK packet base on received packet.
//	Last check is whether this packet is new or old. If new packet
//	recieved, then updte the last rx PID and rx Checksum values for
//	checking next time.
//
//  ARGUMENTS:
//		Header** ppRxHeader
//			Point to Packet header pointer
//		uint8_t* pui8RxBuffer
//			Point to the received packet buffer
//
//  RETURN VALUE:
//      e_HandShakeReturn
//			HANDSHAKERETURN_INVALID		: Invalid packet received...
//			HANDSHAKERETURN_DUPLICATED 	: Duplicated packet received...
//			HANDSHAKERETURN_SUCCESS		: New packet received!
//-----------------------------------------------------------------------------
e_HandShakeReturn Network_isHandShakeProcessSuccess(Header** ppRxHeader, uint8_t* pui8RxBuffer)
{
	// Get packet header using cast method
	*ppRxHeader = (Header*)pui8RxBuffer;

	// Filter invalid packet!
	if((*ppRxHeader)->ui32DestinationAddress == NETWORK_BROADCAST_ADDRESS ||
			(*ppRxHeader)->ui32DestinationAddress == Network_getSelfAddress())
	{
		// Is message packet?
		if((*ppRxHeader)->ui8Length > RF_HEADER_LENGTH)
		{
			// Is ACK require?
			if((*ppRxHeader)->status.eHandShakeType == HANDSHAKE_ACK
					&& (*ppRxHeader)->ui32DestinationAddress != NETWORK_BROADCAST_ADDRESS)
			{
				Network_sendACK(*(*ppRxHeader));
			}

			// Is recieved duplicate packet?
			if((*ppRxHeader)->status.ui4Pid == Network_getLastRxPID() &&
					(*ppRxHeader)->ui16Checksum == Network_getLastRxChecksum())
			{
				return HANDSHAKERETURN_DUPLICATED;
			}

			// New packet received, store the new PID and checksum values
			Network_setLastRxPID((*ppRxHeader)->status.ui4Pid);
			Network_setLastRxChecksum((*ppRxHeader)->ui16Checksum);

			return HANDSHAKERETURN_SUCCESS;
		}
		else // Network packet - hello
		{
			if((*ppRxHeader)->status.ePacketType == PACKET_HELLO)
			{
				Network_sendACK(*(*ppRxHeader));
				return HANDSHAKERETURN_HELLO;
			}
		}
	}
	return HANDSHAKERETURN_INVALID;
}

//-----------------------------------------------------------------------------
//  bool Network::isNextPacket(uint8_t* pRxBuff, va_list argp)
//
//  DESCRIPTION:
//  This function spin until middle or last packet received or
//	MULTIPACKET_TIMEOUT_USEC exprised
//
//  ARGUMENTS:
//		uint8_t* pRxBuff
//			This pointer point to the received packet
//		va_list argp
//			This list containt three argument in order:
//				1/ uint8_t ui8PreviousPID
//					Last success packet PID number
//				2/ uint8_t* pui8RxBuffer
//					Point to the next success packet buffer if success
//				3/ uint8_t* pui8RxMessSize
//					Containt message segment size in the new received packet if success
//
//  RETURN VALUE:
//      bool
//			true	: Success recieved next packet
//			false 	: Retry for another packet
//-----------------------------------------------------------------------------
bool Network_isNextPacket(uint8_t* pRxBuff, va_list argp)
{
	// Get the input arguments
	uint8_t ui8PreviousPID;
	uint8_t* pui8RxBuffer;
	uint8_t* pui8RxMessSize;
	ui8PreviousPID = va_arg(argp, uint8_t);
	pui8RxBuffer = va_arg(argp, uint8_t*);
	pui8RxMessSize = va_arg(argp, uint8_t*);

	// Local vairables
	uint8_t i;
	Header* pRxHeader = 0;
	e_HandShakeReturn eReturn;

	eReturn = Network_isHandShakeProcessSuccess(&pRxHeader, pRxBuff);
	if(eReturn == HANDSHAKERETURN_SUCCESS)
	{
		if (pRxHeader->status.ui4Pid == Network_generateNextPID(ui8PreviousPID)
				&& (pRxHeader->status.ePacketType == PACKET_MIDDLE
						|| pRxHeader->status.ePacketType == PACKET_LAST))
		{
			*pui8RxMessSize = pRxHeader->ui8Length - (RF_HEADER_LENGTH - 1);

			// Passback received message
			for(i = 0; i < RF_PACKET_LENGTH; i++)
				pui8RxBuffer[i] = pRxBuff[i];

			return true;
		}
	}
	return false;
}

//-----------------------------------------------------------------------------
//  void Network::sendACK(Header RxHeader)
//
//  DESCRIPTION:
//  This function generate an ACK packet base on RxHeader and transmit it
//
//  ARGUMENTS:
//      Header RxHeader
//          Request ACK response packet header
//-----------------------------------------------------------------------------
void Network_sendACK(Header RxHeader)
{
	uint8_t i;
	Header AckHeader;
	uint16_t ui16Checksum;

	// Construct Ack packet header
	AckHeader.ui8Length = ACK_PACKET_LENGTH - 1;
	AckHeader.status.eHandShakeType = HANDSHAKE_NOACK;
	AckHeader.status.ePacketType = PACKET_ACK;
	AckHeader.status.ui4Pid = Network_generateNextPID(RxHeader.status.ui4Pid);
	AckHeader.ui32SourceAddress = Network_getSelfAddress();
	AckHeader.ui32DestinationAddress = RxHeader.ui32SourceAddress;
	AckHeader.ui16Checksum = RF_CHECKSUM_PREOFFSET;

	// Generate checksum
	ui16Checksum = 0;
	for(i = 0; i < ACK_PACKET_LENGTH; i++)
		ui16Checksum += *((uint8_t*)(&AckHeader) + i);
	AckHeader.ui16Checksum = ~(ui16Checksum) + 1;

	// Delay before transmit ACK
	RfWaitUs(ACK_PACKET_DELAY_USEC);

	// Transmit ack packet
	RfSendPacket((uint8_t*)(&AckHeader));
}

void Network_deleteBuffer(uint8_t *pui8Buff)
{
	if(pui8Buff != 0)
	{
		delete[] pui8Buff;
		pui8Buff = 0;
	}
}

#endif /* NETWORK_CPP_ */
