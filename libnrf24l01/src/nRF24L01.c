//  ***  modified by VyLong  *** //

#ifdef RF_USE_nRF24L01

#include "libnrf24l01/inc/TM4C123_nRF24L01.h"
#include "libnrf24l01/inc/nRF24L01.h"

// Configuration parameters used to set-up the RF configuration
static uint8_t crcConfig;
static uint8_t addressWidth;

// Status variable updated every time SPI I/O is performed
static uint8_t status;

// Used to track which features have been enabled
static uint8_t features;

//-------------------- Application methods-----------------------------
bool RfSendPacket(uint8_t *txBuffer)
{
	if (txBuffer[0] == 0 || txBuffer[0] > RF24_VALID_BUFFER_SIZE)
		return false;

	uint8_t puiTempBuffer[RF24_BUFFER_SIZE];

	bool bCurrentInterruptStage = MCU_RF_GetInterruptState();
	MCU_RF_DisableInterrupt();

	RF24_TX_activate();

	// Generate 4-byte checksum
	uint32_t ui32Checksum = 0;
	uint8_t i;
	for(i = 1; i <= txBuffer[0]; i++)
	{
		ui32Checksum += txBuffer[i];
		puiTempBuffer[i - 1] = txBuffer[i];
	}
	--i;
	ui32Checksum = (~ui32Checksum) + 1;
	puiTempBuffer[i++] = (uint8_t)(ui32Checksum >> 24);
	puiTempBuffer[i++] = (uint8_t)(ui32Checksum >> 16);
	puiTempBuffer[i++] = (uint8_t)(ui32Checksum >> 8);
	puiTempBuffer[i++] = (uint8_t)ui32Checksum;

	RF24_TX_writePayloadNoAck(txBuffer[0] + RF24_CHECKSUM_SIZE, puiTempBuffer);

	RF24_TX_pulseTransmit();

	while (GPIOPinRead(RF24_INT_PORT, RF24_INT_Pin) != 0);

	while(!RF24_getIrqFlag(RF24_IRQ_TX));

	RF24_clearIrqFlag(RF24_IRQ_MASK);

	MCU_RF_ClearIntFlag();
	MCU_RF_ClearPending();
	if (bCurrentInterruptStage)
		MCU_RF_EnableInterrupt();	// recover last interrupt state

	RfFlushTxFifo();

	RfSetRxMode();

	return true;
}

e_RxStatus RfReceivePacket(uint8_t *rxBuffer)
{
	uint8_t i;
	uint8_t ui8RxLength;

	if (RF24_getIrqFlag(RF24_IRQ_RX))
	{
		ui8RxLength = RF24_RX_getPayloadWidth();

		if (ui8RxLength > RF24_BUFFER_SIZE)
		{
			RF24_clearIrqFlag(RF24_IRQ_MASK);

			RfFlushRxFifo();

			return RX_STATUS_INVALID_LENGTH;
		}

		RF24_RX_getPayloadData(ui8RxLength, &rxBuffer[1]);

		uint32_t ui32Checksum = (rxBuffer[ui8RxLength - 3] << 24) |
								(rxBuffer[ui8RxLength - 2] << 16) |
								(rxBuffer[ui8RxLength - 1] << 8) |
								rxBuffer[ui8RxLength];

		rxBuffer[0] = ui8RxLength - RF24_CHECKSUM_SIZE;

		for(i = 1; i <= rxBuffer[0]; i++)
		{
			ui32Checksum += rxBuffer[i];
		}

		RF24_clearIrqFlag(RF24_IRQ_RX);

		if(ui32Checksum == 0)
			return RX_STATUS_SUCCESS;
		else
			return RX_STATUS_CRC_ERROR;
	}
	else
	{
		RF24_clearIrqFlag(RF24_IRQ_MASK);

		return RX_STATUS_FAILED;
	}
}
void initRfModule(bool isEnableInt)
{
	RF24_InitTypeDef initRf24;
	initRf24.AddressWidth = RF24_ADRESS_WIDTH_3;
	initRf24.Channel = RF24_CHANNEL_51;
	initRf24.CrcBytes = RF24_CRC_2BYTES;
	initRf24.CrcState = RF24_CRC_DIS;
	initRf24.RetransmitCount = RF24_RETRANS_DISABLE;
	initRf24.RetransmitDelay = RF24_RETRANS_DELAY_250u;
	initRf24.Speed = RF24_SPEED_1MBPS;
	initRf24.Power = RF24_POWER_0DBM;
	initRf24.Features = RF24_FEATURE_EN_DYNAMIC_PAYLOAD
			| RF24_FEATURE_EN_NO_ACK_COMMAND;
	initRf24.InterruptEnable = isEnableInt;
	initRf24.LNAGainEnable = true;
	RF24_init(&initRf24);

	// Set 2 pipes dynamic payload
	RF24_PIPE_setPacketSize(RF24_PIPE0, RF24_PACKET_SIZE_DYNAMIC);

	// Open pipe#0 without Enhanced ShockBurst
	RF24_PIPE_open(RF24_PIPE0, true);
	RF24_PIPE_open(RF24_PIPE1, false);
	RF24_PIPE_open(RF24_PIPE2, false);
	RF24_PIPE_open(RF24_PIPE3, false);
	RF24_PIPE_open(RF24_PIPE4, false);
	RF24_PIPE_open(RF24_PIPE5, false);

	RF24_PIPE_close(RF24_PIPE1);
	RF24_PIPE_close(RF24_PIPE2);
	RF24_PIPE_close(RF24_PIPE3);
	RF24_PIPE_close(RF24_PIPE4);
	RF24_PIPE_close(RF24_PIPE5);

	uint8_t addr[3];

	addr[2] = RF24_DEFAULT_ADDRESS_BYTE2;
	addr[1] = RF24_DEFAULT_ADDRESS_BYTE1;
	addr[0] = RF24_DEFAULT_ADDRESS_BYTE0;
	RF24_RX_setAddress(RF24_PIPE0, addr);
	RF24_TX_setAddress(addr);

	RfSetRxMode();
}
void RfSetChannel(uint8_t chanNum)
{
	RF24_writeRegister(RF24_REG_RF_CH, (chanNum & 0x7F));
}
void RfSetRxMode(void)
{
	RF24_startStandby();
	RfFlushRxFifo();
	RF24_clearIrqFlag(RF24_IRQ_RX);
	RF24_setConfigureRegister(crcConfig | RF24_PWR_UP | RF24_PRIM_RX);
	MCU_RF_SetCE();
}
void RfFlushTxFifo(void)
{
	MCU_RF_ClearCSN();
	status = MCU_RF_SendAndGetData(RF24_COMMAND_FLUSH_TX);
	MCU_RF_SetCSN();
}
void RfFlushRxFifo(void)
{
	MCU_RF_ClearCSN();
	status = MCU_RF_SendAndGetData(RF24_COMMAND_FLUSH_RX);
	MCU_RF_SetCSN();
}

void RfWaitUs(uint32_t periodUs)
{
	MCU_RF_WaitUs(periodUs);
}

//-----------------------------------------------------------------------------
//  bool RfTryToGetRxPacket(uint64_t ui64PeriodInUs,
//				bool (*pfnDecodePacket)(uint8_t* pRxBuff, va_list argp), ...)
//
//  DESCRIPTION:
//	This function is used for non-interrupt Rx packet handle. The process is periodic
//	check for the Rf interrupt pin to detect new packet received. This loop will expired in
//	ui64PeriodInUs microsecond. If a packet is recieved, the packer decoder will be call.
//  If a correct packet is received (return form decoder function) the process terminal immedialy.
//
//  ARGUMENTS:
//		uint64_t ui64PeriodInUs
//			The period in microsecond for this function expired
//		bool (*pfnDecodePacket)(uint8_t* pRxBuff, va_list argp)
//			Packet decoder function pointer
//		...
//			Variables argument for the pfnDecodePacket function
//
//  RETURN VALUE:
//      bool
//          true	: Received expected packet, decoder function return true
//			false	: Process expired, not received correct packet
//-----------------------------------------------------------------------------
bool RfTryToGetRxPacket(uint64_t ui64PeriodInUs,
		bool (*pfnDecodePacket)(uint8_t* pRxBuff, va_list argp), ...)
{
	bool bCurrentInterruptStage = MCU_RF_GetInterruptState();
	MCU_RF_DisableInterrupt();

	bool bReturn = false;

	va_list argp;
	va_start(argp, pfnDecodePacket);	// Start the varargs processing.

	uint64_t i;
	uint8_t pui8RxBuffer[TXBUFFERSIZE] =
	{ 0 };
	for (i = 0; i < ui64PeriodInUs; i++)
	{
		if (MCU_RF_IsInterruptPinAsserted())
		{
			MCU_RF_ClearIntFlag();

			if (RfReceivePacket(pui8RxBuffer) == RX_STATUS_SUCCESS)
			{   // Fetch packet from CCxxxx
				// Call packet decoder
				if ((*pfnDecodePacket)(pui8RxBuffer, argp))
				{
					// if decode success then terminal this process
					bReturn = true;
					break;
				}
			}
		}
		MCU_RF_WaitUs(1); // delay for 1us
	}

	va_end(argp);	// We're finished with the varargs now.

	MCU_RF_ClearIntFlag();
	MCU_RF_ClearPending();
	if (bCurrentInterruptStage)
		MCU_RF_EnableInterrupt();	// recover last interrupt state

	return bReturn;
}

bool RfTryToCaptureRfSignal(uint64_t ui64PeriodInUs,
			bool (*pfnHandler)(va_list argp), ...)
{
	bool bCurrentInterruptStage = MCU_RF_GetInterruptState();
	MCU_RF_DisableInterrupt();

	bool bReturn = false;

	va_list argp;
	va_start(argp, pfnHandler);	// Start the varargs processing.

	uint64_t i;
	for(i = 0; i < ui64PeriodInUs; i++)
	{
		if (MCU_RF_IsInterruptPinAsserted())
		{
			MCU_RF_ClearIntFlag();

			// Call handler
			if((*pfnHandler)(argp))
			{
				// if decode success then terminal this process
				bReturn = true;
				break;
			}
		}
		MCU_RF_WaitUs(1); // delay for 1us
	}

	va_end(argp);	// We're finished with the varargs now.

	// Because previous action may have assert interrupt flag
    MCU_RF_ClearIntFlag();
    MCU_RF_ClearPending();
	if (bCurrentInterruptStage)
		MCU_RF_EnableInterrupt();	// recover last interrupt state

	return bReturn;
}


//----------------------- nRF24L01 library -----------------------------

void RF24_init(const RF24_InitTypeDef* InitRf24)
{
	MCU_RF_InitSpiForRf();

	MCU_RF_ConfigIRQPin(InitRf24->InterruptEnable);

	MCU_RF_SetCSN();
	MCU_RF_ClearCE();

	MCU_RF_WaitUs(5000);

	RF24_clearIrqFlag(RF24_IRQ_MASK);

	RF24_PIPE_closeAll();

	RF24_RETRANS_setDelay(InitRf24->RetransmitDelay);
	RF24_RETRANS_setCount(InitRf24->RetransmitCount);
	RF24_setSpeedAndPower(
			InitRf24->Speed | InitRf24->Power | InitRf24->LNAGainEnable);
	RfSetChannel(InitRf24->Channel);
	RF24_setAddressWidth(InitRf24->AddressWidth);

	features = 0x00;
	RF24_enableFeatures(InitRf24->Features);

	RfFlushTxFifo();
	RfFlushRxFifo();

	crcConfig = InitRf24->CrcState | InitRf24->CrcBytes;
	addressWidth = InitRf24->AddressWidth + 2;
}

void RF24_enableFeatures(uint8_t enFeature)
{
	if ((features & enFeature) != enFeature)
	{
		features |= enFeature;
		features &= 0x07;
		RF24_writeRegister(RF24_REG_FEATURE, features);
	}
}

void RF24_disableFeatures(uint8_t disfeature)
{
	if (features & disfeature)
	{
		features &= ~disfeature;
		RF24_writeRegister(RF24_REG_FEATURE, features);
	}
}

uint8_t RF24_getLostPackets()
{
	return (RF24_readRegister(RF24_REG_OBSERVE_TX) >> 4) & 0x0F;
}

uint8_t RF24_isAlive()
{
	uint8_t aw;

	aw = RF24_readRegister(RF24_REG_SETUP_AW);
	return ((aw & 0xFC) == 0x00 && (aw & 0x03) != 0x00);
}

void RF24_setConfigureRegister(uint8_t newConfigure)
{
	RF24_writeRegister(RF24_REG_CONFIG, (newConfigure & RF24_REG_CONFIG_MASK));
}

uint8_t RF24_getConfigureRegister()
{
	return RF24_readRegister(RF24_REG_CONFIG);
}

void RF24_setSpeedAndPower(uint8_t rfSpeedAndPower)
{
	//rfSpeedAndPower &= 0x0F;
	RF24_writeRegister(RF24_REG_RF_SETUP, rfSpeedAndPower);
}

void RF24_setAddressWidth(uint8_t rfAddressWidth)
{
	rfAddressWidth &= 0x03;
	RF24_writeRegister(RF24_REG_SETUP_AW, rfAddressWidth);
}

uint8_t RF24_getCurrentState()
{
	uint8_t config;

	if (!RF24_isAlive())
		return RF24_STATE_NOTPRESENT;

	config = RF24_readRegister(RF24_REG_CONFIG);
	if ((config & RF24_PWR_UP) == RF24_PWR_DOWN)
		return RF24_STATE_POWERDOWN;

	if (MCU_RF_GetCEState() == 0)
		return RF24_STATE_STANDBY_I;

	if (config & RF24_PRIM_RX)
	{
		// Testing CONT or PLL_LOCK?
		if (RF24_readRegister(RF24_REG_RF_SETUP) & 0x90)
			return RF24_STATE_TEST;

		// PWR_UP=1, PRIM_RX=1, CE=1 -- Must be PRX
		return RF24_STATE_PRX;
	}

	// TX FIFO empty?
	if ((RF24_readRegister(RF24_REG_FIFO_STATUS) & RF24_TX_EMPTY))
		return RF24_STATE_STANDBY_II;

	// If TX FIFO is not empty, we are in PTX (active transmit) mode.
	return RF24_STATE_PTX;
}

void RF24_startPowerDown()
{
	MCU_RF_ClearCE();
	RF24_setConfigureRegister(crcConfig | RF24_PWR_DOWN | RF24_PRIM_RX);
}

void RF24_startStandby()
{
	uint8_t config = RF24_readRegister(RF24_REG_CONFIG);

	MCU_RF_ClearCE();

	if ((config & RF24_PWR_UP) == RF24_PWR_DOWN)
	{
		if (config & RF24_PRIM_RX)
			RF24_setConfigureRegister(crcConfig | RF24_PWR_UP | RF24_PRIM_RX);
		else
			RF24_setConfigureRegister(crcConfig | RF24_PWR_UP | RF24_PRIM_TX);

		MCU_RF_WaitUs(5000);
	}
}

uint8_t RF24_scan()
{
	int16_t testcount = 1023;
	uint8_t rpdcount = 0;
	uint8_t last_state;

	last_state = RF24_getCurrentState();
	if (last_state != RF24_STATE_PRX)
		RfSetRxMode();

	for (; testcount > 0; testcount--)
	{
		if (RF24_readRegister(RF24_REG_RPD))
			rpdcount++;
		MCU_RF_WaitUs(130);
		RfFlushRxFifo();
		RF24_clearIrqFlag(RF24_IRQ_RX);
	}

	if (last_state != RF24_STATE_PRX)
		RF24_startStandby();
	return ((uint8_t) (rpdcount >> 2));
}

uint8_t RF24_getIrqFlag(uint8_t rfIrqFlag)
{
	MCU_RF_ClearCSN();
	status = MCU_RF_SendAndGetData(RF24_COMMAND_NOP);
	MCU_RF_SetCSN();
	status &= RF24_IRQ_MASK;
	status &= rfIrqFlag;
	return status;
}

void RF24_clearIrqFlag(uint8_t irqflag)
{
	MCU_RF_ClearCSN();
	status = MCU_RF_SendAndGetData(RF24_REG_STATUS | RF24_COMMAND_W_REGISTER);
	status &= 0x0F;
	irqflag |= status;
	MCU_RF_SendAndGetData(irqflag);
	MCU_RF_SetCSN();
}

uint8_t RF24_readRegister(uint8_t addr)
{
	uint8_t i;
	MCU_RF_ClearCSN();
	status = MCU_RF_SendAndGetData(
			(addr & RF24_REG_MASK) | RF24_COMMAND_R_REGISTER);
	i = MCU_RF_SendAndGetData(RF24_COMMAND_NOP);
	MCU_RF_SetCSN();
	return i;
}

void RF24_writeRegister(uint8_t addr, uint8_t data)
{
	MCU_RF_ClearCSN();
	status = MCU_RF_SendAndGetData(
			(addr & RF24_REG_MASK) | RF24_COMMAND_W_REGISTER);
	MCU_RF_SendAndGetData(data);
	MCU_RF_SetCSN();
}

void RF24_TX_setAddress(uint8_t *addr)
{
	int8_t i;
	MCU_RF_ClearCSN();
	status = MCU_RF_SendAndGetData(RF24_REG_TX_ADDR | RF24_COMMAND_W_REGISTER);
	for (i = 0; i < addressWidth; i++)
	{
		MCU_RF_SendAndGetData(addr[i]);
	}
	MCU_RF_SetCSN();
}

void RF24_TX_writePayloadAck(uint8_t len, uint8_t *data)
{
	uint8_t i = 0;
	MCU_RF_ClearCSN();
	status = MCU_RF_SendAndGetData(RF24_COMMAND_W_TX_PAYLOAD);
	for (i = 0; i < len; i++)
	{
		MCU_RF_SendAndGetData(data[i]);
	}
	MCU_RF_SetCSN();
}

void RF24_TX_writePayloadNoAck(uint8_t len, uint8_t *data)
{
	uint8_t i = 0;
	if (!(features & RF24_FEATURE_EN_NO_ACK_COMMAND))
		return;
	MCU_RF_ClearCSN();
	status = MCU_RF_SendAndGetData(RF24_COMMAND_W_TX_PAYLOAD_NOACK);
	for (i = 0; i < len; i++)
	{
		MCU_RF_SendAndGetData(data[i]);
	}
	MCU_RF_SetCSN();
}

void RF24_TX_reuseLastPayload()
{
	MCU_RF_ClearCSN();
	status = MCU_RF_SendAndGetData(RF24_COMMAND_REUSE_TX_PL);
	MCU_RF_SetCSN();
}

void RF24_TX_activate()
{
	RF24_startStandby();
	RfFlushTxFifo();
	RF24_setConfigureRegister(crcConfig | RF24_PWR_UP | RF24_PRIM_TX);
	RF24_clearIrqFlag(RF24_IRQ_TX | RF24_IRQ_MAX_RETRANS);
	MCU_RF_ClearCE();
}

void RF24_TX_pulseTransmit()
{
	MCU_RF_SetCE();
	MCU_RF_WaitUs(15);
	MCU_RF_ClearCE();
}

void RF24_RX_setAddress(uint8_t pipe, uint8_t *addr)
{
	int8_t i;
	if (pipe > 5)
		return;

	MCU_RF_ClearCSN();
	status = MCU_RF_SendAndGetData(
			(RF24_REG_RX_ADDR_P0 + pipe) | RF24_COMMAND_W_REGISTER);
	if (pipe > 1)
	{
		// Pipes 2-5 differ from pipe1's addr only in the Least Significant Byte.
		MCU_RF_SendAndGetData(addr[0]);
	}
	else
	{
		for (i = 0; i < addressWidth; i++)
		{
			MCU_RF_SendAndGetData(addr[i]);
		}
	}
	MCU_RF_SetCSN();
}

uint8_t RF24_RX_getPayloadWidth()
{
	uint8_t i;
	MCU_RF_ClearCSN();
	status = MCU_RF_SendAndGetData(RF24_COMMAND_R_RX_PL_WID);
	i = MCU_RF_SendAndGetData(RF24_COMMAND_NOP);
	MCU_RF_SetCSN();
	return i;
}

uint8_t RF24_RX_getPayloadData(uint8_t len, uint8_t *data)
{
	uint8_t i;
	MCU_RF_ClearCSN();
	status = MCU_RF_SendAndGetData(RF24_COMMAND_R_RX_PAYLOAD);
	for (i = 0; i < len; i++)
	{
		data[i] = MCU_RF_SendAndGetData(RF24_COMMAND_NOP);
	}
	MCU_RF_SetCSN();

	// The RX pipe this data belongs to is stored in STATUS
	return ((status & 0x0E) >> 1);
}

uint8_t RF24_RX_isEmpty()
{
	uint8_t fifoStatus;

	fifoStatus = RF24_readRegister(RF24_REG_FIFO_STATUS);

	if (fifoStatus & RF24_RX_EMPTY)
		return 1;

	return 0;
}

void RF24_RX_sendAckWithPayload(uint8_t pipe, uint8_t len, uint8_t *data)
{
	uint8_t i;

	if (pipe > 5)
		return;

	if (!(features & RF24_FEATURE_EN_ACK_WITH_PAYLOAD))
		return;

	MCU_RF_ClearCSN();
	status = MCU_RF_SendAndGetData(RF24_COMMAND_W_ACK_PAYLOAD | pipe);
	for (i = 0; i < len; i++)
	{
		MCU_RF_SendAndGetData(data[i]);
	}
	MCU_RF_SetCSN();
}

bool RF24_RX_carrierDetection()
{
	uint8_t CD;

	CD = RF24_readRegister(RF24_REG_CD);
	return (CD & 0x01);
}

void RF24_PIPE_close(uint8_t pipeid)
{
	uint8_t rxen, enaa;
	if (pipeid > 5)
		return;

	rxen = RF24_readRegister(RF24_REG_EN_RXADDR);
	enaa = RF24_readRegister(RF24_REG_EN_AA);

	rxen &= ~(1 << pipeid);
	enaa &= ~(1 << pipeid);

	RF24_writeRegister(RF24_REG_EN_RXADDR, rxen);
	RF24_writeRegister(RF24_REG_EN_AA, enaa);
}

void RF24_PIPE_closeAll()
{
	RF24_writeRegister(RF24_REG_EN_RXADDR, 0x00);
	RF24_writeRegister(RF24_REG_EN_AA, 0x00);
	RF24_writeRegister(RF24_REG_DYNPD, 0x00);
}

void RF24_PIPE_open(uint8_t pipe, bool autoack)
{
	uint8_t rxen, enaa;

	rxen = RF24_readRegister(RF24_REG_EN_RXADDR);
	enaa = RF24_readRegister(RF24_REG_EN_AA);

	if (autoack)
		enaa |= (1 << pipe);
	else
		enaa &= ~(1 << pipe);

	rxen |= (1 << pipe);
	RF24_writeRegister(RF24_REG_EN_RXADDR, rxen);
	RF24_writeRegister(RF24_REG_EN_AA, enaa);
}

uint8_t RF24_PIPE_isOpen(uint8_t pipeid)
{
	uint8_t rxen;
	if (pipeid > 5)
		return 0;

	rxen = RF24_readRegister(RF24_REG_EN_RXADDR);

	return ((1 << pipeid) == (rxen & (1 << pipeid)));
}

void RF24_PIPE_setPacketSize(uint8_t pipe, uint8_t size)
{
	uint8_t dynpdcfg;

	dynpdcfg = RF24_readRegister(RF24_REG_DYNPD);
	if (size < 1)
	{
		// Cannot set dynamic payload if EN_DPL is disabled.
		if (!(features & RF24_FEATURE_EN_DYNAMIC_PAYLOAD))
			return;
		if (!((1 << pipe) & dynpdcfg))
		{
			// Dynamic payload not enabled for this pipe, enable it
			dynpdcfg |= (1 << pipe);
		}
	}
	else
	{
		// Ensure DynPD is disabled for this pipe
		dynpdcfg &= ~(1 << pipe);
		if (size > 32)
			size = 32;
		RF24_writeRegister(RF24_REG_RX_PW_P0 + pipe, size);
	}
	RF24_writeRegister(RF24_REG_DYNPD, dynpdcfg);
}

void RF24_RETRANS_setDelay(uint8_t delay)
{
	uint8_t retransRegsiterValue;
	delay &= 0xF0;
	retransRegsiterValue = RF24_readRegister(RF24_REG_SETUP_RETR);
	retransRegsiterValue &= 0x0F;
	RF24_writeRegister(RF24_REG_SETUP_RETR, delay | retransRegsiterValue);
}

void RF24_RETRANS_setCount(uint8_t count)
{
	uint8_t retransRegsiterValue;
	count &= 0x0F;
	retransRegsiterValue = RF24_readRegister(RF24_REG_SETUP_RETR);
	retransRegsiterValue &= 0xF0;
	RF24_writeRegister(RF24_REG_SETUP_RETR, count | retransRegsiterValue);
}

uint8_t RF24_RETRANS_getLastRetransmits(void)
{
	return RF24_readRegister(RF24_REG_OBSERVE_TX) & 0x0F;
}

#endif
