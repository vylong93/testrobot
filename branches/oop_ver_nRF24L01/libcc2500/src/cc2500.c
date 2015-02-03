//----------------------------------------------------------------------------
//  Description:  This file contains functions that configure the CC1100/2500
//  device.
//
//  Demo Application for MSP430/CC1100-2500 Interface Code Library v1.0
//
//  Randy Wu
//  Texas Instruments, Inc.
//  May 2007
//  IAR Embedded Workbench v3.42A

//----------------------------------------------------------------------------

#include "libcc2500/inc/cc2500.h"
#include "libcc2500/inc/TM4C123_CC2500.h"


//uint8_t freqTab[RFNUMCHANS][3] = { {0xB1, 0x53, 0x5C},  // 2.4005 GHz
//                                   {0x13, 0x3B, 0x5D},  // 2.4240 GHz
//                                   {0x13, 0xBB, 0x5D},  // 2.4370 GHz
//                                   {0x76, 0x62, 0x5F},  // 2.4800 GHz
//                                  };


//-----------------------------------------------------------------------------
// void RfWriteSettings(void)
//
//  DESCRIPTION:
//  Used to configure the CCxxxx registers.  There are five instances of this
//  function, one for each available carrier frequency.  The instance compiled
//  is chosen according to the system variable TI_CC_RF_FREQ, assigned within
//  the header file "TI_CC_hardware_board.h".
//
//  ARGUMENTS:
//      none
//
// Product = CC2500
// Crystal accuracy = 40 ppm
// X-tal frequency = 26 MHz
// RF output power = 0 dBm
// RX filterbandwidth = 812.500000 kHz
// Deviation = 0.000000
// Return state:  Return to Idle state upon leaving TX either RX
// Datarate = 500.000000 kbps
// Modulation = (7) MSK
// Manchester enable = (0) Manchester disabled
// RF Frequency = 2433.000000 MHz
// Channel spacing = 199.950000 kHz
// Channel number = 0
// Optimization = Sensitivity
// Sync mode = (3) 30/32 sync word bits detected
// Format of RX/TX data = (0) Normal mode, use FIFOs for RX and TX
// CRC operation = (1) CRC calculation in TX and CRC check in RX enabled
// Forward Error Correction = (0) FEC disabled
// Length configuration = (1) Variable length packets, packet length configured by the first received byte after sync word.
// Packetlength = 255
// Preamble count = (2)  4 bytes
// Append status = 0
// Address check = (0) No address check
// FIFO autoflush = 0
// Device address = 0
// GDO0 signal selection = (06) Asserts when sync word has been sent / received, and de-asserts at the end of the packet
// GDO2 signal selection = (15) CRC OK
void RfWriteSettings(void)
{
    // Write register settings
    RfWriteReg(TI_CCxxx0_IOCFG2,   0x0F);  // GDO2 output pin config.
    RfWriteReg(TI_CCxxx0_IOCFG0,   0x06);  // GDO0 output pin config.
    RfWriteReg(TI_CCxxx0_PKTLEN,   0xFF);  // Packet length.
    RfWriteReg(TI_CCxxx0_PKTCTRL1, 0x00);  // Packet automation control.
    RfWriteReg(TI_CCxxx0_PKTCTRL0, 0x0D);  // *Packet automation control.
    RfWriteReg(TI_CCxxx0_ADDR,     0x00);  // Device address.
    RfWriteReg(TI_CCxxx0_CHANNR,   0x00); // Channel number.
    RfWriteReg(TI_CCxxx0_FSCTRL1,  0x0C); // Freq synthesizer control.
    RfWriteReg(TI_CCxxx0_FSCTRL0,  0x00); // Freq synthesizer control.
    RfWriteReg(TI_CCxxx0_FREQ2,    0x5D); // Freq control word, high byte
    RfWriteReg(TI_CCxxx0_FREQ1,    0x93); // Freq control word, mid byte.
    RfWriteReg(TI_CCxxx0_FREQ0,    0xB1); // Freq control word, low byte.
    RfWriteReg(TI_CCxxx0_MDMCFG4,  0x0E); // Modem configuration.
    RfWriteReg(TI_CCxxx0_MDMCFG3,  0x3B); // Modem configuration.
    RfWriteReg(TI_CCxxx0_MDMCFG2,  0x73); // Modem configuration.
    RfWriteReg(TI_CCxxx0_MDMCFG1,  0x42); // Modem configuration.
    RfWriteReg(TI_CCxxx0_MDMCFG0,  0xF8); // Modem configuration.
    RfWriteReg(TI_CCxxx0_DEVIATN,  0x00); // Modem dev (when FSK mod en)
#ifdef _SWOR
    // Set Event0 timeout = 300 ms (RX polling interval)
    // WOR_RES = 0
    // T_event0 = 750 / f_xosc * EVENT0 * 2^(5*WOR_RES) = 300 ms    // Assuming f_xosc = 26 MHz
    // =>  EVENT0 = 10400 = 0x28A0
    RfWriteReg(TI_CCxxx0_WOREVT1,  0x28); // WOR High byte Event Timeout.
    RfWriteReg(TI_CCxxx0_WOREVT0,  0xA0); // WOR Low byte Event Timeout.
    // Enable automatic initial calibration of RCosc.
    // Set T_event1 ~ 345 us, enough for XOSC stabilize before starting calibration.
    // Enable RC oscillator before starting with WOR (or else it will not wake up).
    // Not using AUTO_SYNC function.
    RfWriteReg(TI_CCxxx0_WORCTRL,  0x38); // WOR control
    // Setting Rx_timeout < 0.5 %.
    // => MCSM2.RX_TIME = 101b
    // => Rx_timeout = (EVENT0 * 0.1127) = (10400 * 0.1127) =  1.172 ms, i.e.  0.391% RX duty cycle
    // => MCSM2.RX_TIME = 000b
    // => Rx_timeout = (EVENT0 * 3.6029) = (10400 * 3.6029) = 37.471 ms, i.e. 12.500% RX duty cycle
    RfWriteReg(TI_CCxxx0_MCSM2,    0x00); // MainRadio Cntrl State Machine
#endif /* _SWOR */
    // RXOFF_MODE=01b (RX->FSTXON: 9.6 us), TXOFF_MODE=00b (TX->IDLE, no FS calib: 0.1 us).
    RfWriteReg(TI_CCxxx0_MCSM1,    0x30); // MainRadio Cntrl State Machine
    RfWriteReg(TI_CCxxx0_MCSM0,    0x18); // MainRadio Cntrl State Machine
    RfWriteReg(TI_CCxxx0_FOCCFG,   0x1D); // Freq Offset Compens. Config
    RfWriteReg(TI_CCxxx0_BSCFG,    0x1C); // Bit synchronization config.
    RfWriteReg(TI_CCxxx0_AGCCTRL2, 0xC7); // AGC control.
    RfWriteReg(TI_CCxxx0_AGCCTRL1, 0x40); // AGC control.
    RfWriteReg(TI_CCxxx0_AGCCTRL0, 0xB0); // AGC control.
    RfWriteReg(TI_CCxxx0_FREND1,   0xB6); // Front end RX configuration.
    RfWriteReg(TI_CCxxx0_FREND0,   0x10); // Front end RX configuration.
    RfWriteReg(TI_CCxxx0_FSCAL3,   0xEA); // Frequency synthesizer cal.
    RfWriteReg(TI_CCxxx0_FSCAL2,   0x0A); // Frequency synthesizer cal.
    RfWriteReg(TI_CCxxx0_FSCAL1,   0x00); // Frequency synthesizer cal.
    RfWriteReg(TI_CCxxx0_FSCAL0,   0x19); // Frequency synthesizer cal.
    RfWriteReg(TI_CCxxx0_FSTEST,   0x59); // Frequency synthesizer cal.
    RfWriteReg(TI_CCxxx0_TEST2,    0x88); // Various test settings.
    RfWriteReg(TI_CCxxx0_TEST1,    0x31); // Various test settings.
    RfWriteReg(TI_CCxxx0_TEST0,    0x0B); // Various test settings.

//    // Write register settings
//    RfWriteReg(TI_CCxxx0_IOCFG2,   0x0F);  // GDO2 output pin config.
//    RfWriteReg(TI_CCxxx0_IOCFG0,   0x06);  // GDO0 output pin config.
//    RfWriteReg(TI_CCxxx0_PKTLEN,   0xFF);  // Packet length.
//    RfWriteReg(TI_CCxxx0_PKTCTRL1, 0x00);  // Packet automation control.
//    RfWriteReg(TI_CCxxx0_PKTCTRL0, 0x0D);  // *Packet automation control.
//    RfWriteReg(TI_CCxxx0_ADDR,     0x00);  // Device address.
//    RfWriteReg(TI_CCxxx0_CHANNR,   0x00); // Channel number.
//    RfWriteReg(TI_CCxxx0_FSCTRL1,  0x12); // Freq synthesizer control.
//    RfWriteReg(TI_CCxxx0_FSCTRL0,  0x00); // Freq synthesizer control.
//    RfWriteReg(TI_CCxxx0_FREQ2,    0x5C); // Freq control word, high byte
//    RfWriteReg(TI_CCxxx0_FREQ1,    0x53); // Freq control word, mid byte.
//    RfWriteReg(TI_CCxxx0_FREQ0,    0xB1); // Freq control word, low byte.
//    RfWriteReg(TI_CCxxx0_MDMCFG4,  0x2D); // Modem configuration.
//    RfWriteReg(TI_CCxxx0_MDMCFG3,  0x3B); // Modem configuration.
//    RfWriteReg(TI_CCxxx0_MDMCFG2,  0xF3); // Modem configuration.
//    RfWriteReg(TI_CCxxx0_MDMCFG1,  0x22); // Modem configuration.
//    RfWriteReg(TI_CCxxx0_MDMCFG0,  0xF8); // Modem configuration.
//    RfWriteReg(TI_CCxxx0_DEVIATN,  0x00); // Modem dev (when FSK mod en)
//#ifdef _SWOR
//    // Set Event0 timeout = 300 ms (RX polling interval)
//    // WOR_RES = 0
//    // T_event0 = 750 / f_xosc * EVENT0 * 2^(5*WOR_RES) = 300 ms    // Assuming f_xosc = 26 MHz
//    // =>  EVENT0 = 10400 = 0x28A0
//    RfWriteReg(TI_CCxxx0_WOREVT1,  0x28); // WOR High byte Event Timeout.
//    RfWriteReg(TI_CCxxx0_WOREVT0,  0xA0); // WOR Low byte Event Timeout.
//    // Enable automatic initial calibration of RCosc.
//    // Set T_event1 ~ 345 us, enough for XOSC stabilize before starting calibration.
//    // Enable RC oscillator before starting with WOR (or else it will not wake up).
//    // Not using AUTO_SYNC function.
//    RfWriteReg(TI_CCxxx0_WORCTRL,  0x38); // WOR control
//    // Setting Rx_timeout < 0.5 %.
//    // => MCSM2.RX_TIME = 101b
//    // => Rx_timeout = (EVENT0 * 0.1127) = (10400 * 0.1127) =  1.172 ms, i.e.  0.391% RX duty cycle
//    // => MCSM2.RX_TIME = 000b
//    // => Rx_timeout = (EVENT0 * 3.6029) = (10400 * 3.6029) = 37.471 ms, i.e. 12.500% RX duty cycle
//    RfWriteReg(TI_CCxxx0_MCSM2,    0x00); // MainRadio Cntrl State Machine
//#endif /* _SWOR */
//    // RXOFF_MODE=01b (RX->FSTXON: 9.6 us), TXOFF_MODE=00b (TX->IDLE, no FS calib: 0.1 us).
//    RfWriteReg(TI_CCxxx0_MCSM1,    0x30); // MainRadio Cntrl State Machine
//    RfWriteReg(TI_CCxxx0_MCSM0,    0x18); // MainRadio Cntrl State Machine
//    RfWriteReg(TI_CCxxx0_FOCCFG,   0x1D); // Freq Offset Compens. Config
//    RfWriteReg(TI_CCxxx0_BSCFG,    0x1C); // Bit synchronization config.
//    RfWriteReg(TI_CCxxx0_AGCCTRL2, 0xC7); // AGC control.
//    RfWriteReg(TI_CCxxx0_AGCCTRL1, 0x00); // AGC control.
//    RfWriteReg(TI_CCxxx0_AGCCTRL0, 0xB0); // AGC control.
//    RfWriteReg(TI_CCxxx0_FREND1,   0xB6); // Front end RX configuration.
//    RfWriteReg(TI_CCxxx0_FREND0,   0x10); // Front end RX configuration.
//    RfWriteReg(TI_CCxxx0_FSCAL3,   0xEA); // Frequency synthesizer cal.
//    RfWriteReg(TI_CCxxx0_FSCAL2,   0x0A); // Frequency synthesizer cal.
//    RfWriteReg(TI_CCxxx0_FSCAL1,   0x00); // Frequency synthesizer cal.
//    RfWriteReg(TI_CCxxx0_FSCAL0,   0x11); // Frequency synthesizer cal.
//    RfWriteReg(TI_CCxxx0_FSTEST,   0x59); // Frequency synthesizer cal.
//    RfWriteReg(TI_CCxxx0_TEST2,    0x88); // Various test settings.
//    RfWriteReg(TI_CCxxx0_TEST1,    0x31); // Various test settings.
//    RfWriteReg(TI_CCxxx0_TEST0,    0x0B); // Various test settings.
}

//// PATABLE (0 dBm output power)
//extern uint8_t paTable[] = {0xFB}; // Table 31 - Page 47 of 89
//extern uint8_t paTableLen = 1;

//-----------------------------------------------------------------------------
//  bool RfSendPacket(uint8_t *txBuffer, uint8_t size)
//
//  DESCRIPTION:
//  This function transmits a packet with length up to 63 bytes.  To use this
//  function, GD00 must be configured to be asserted when sync word is sent and
//  de-asserted at the end of the packet, which is accomplished by setting the
//  IOCFG0 register to 0x06, per the CCxxxx datasheet.  GDO0 goes high at
//  packet start and returns low when complete.  The function polls GDO0 to
//  ensure packet completion before returning.
//
//  ARGUMENTS:
//      uint8_t *txBuffer
//          Pointer to a buffer containing the data to be transmitted
//
//      uint8_t size
//          The size of the txBuffer
//
//  RETURN VALUE:
//      bool
//          true	: Tx success!
//			false	: Tx failed...
//-----------------------------------------------------------------------------
bool RfSendPacket(uint8_t *txBuffer, uint8_t size)
{
  if(txBuffer[0] == 0)
	  return false;

  bool bCurrentInterruptStage = TI_CC_GetInterruptState();
  TI_CC_DisableInterrupt();

  //TODO: use Tx-if-CCA instead and return true if Tx success
//  while((TI_CC_Strobe(TI_CCxxx0_STX) & TI_CCxxx0_STATE_MASK) != TI_CCxxx0_STATE_TX);

  TI_CC_Strobe(TI_CCxxx0_SIDLE);
  RfWriteBurstReg(TI_CCxxx0_TXFIFO, txBuffer, size); // Write TX data

  // The CC1100 won't transmit the contents of the FIFO until the state is
  // changed to TX state.  During configuration we placed it in RX state and
  // configured it to return to RX whenever it is done transmitting, so it is
  // in RX now.  Use the appropriate library function to change the state to TX.
  TI_CC_Strobe(TI_CCxxx0_STX);           	// Change state to TX, initiating
                                            // data transfer

  TI_CC_WaitForIntGoHigh();		// Wait GDO0 to go hi -> sync TX'ed
  TI_CC_WaitForIntGoLow();		// Wait GDO0 to clear -> end of pkt

  TI_CC_ClearIntFlag();			// Because previous action may have assert interrupt flag
  TI_CC_ClearPending();

  if (bCurrentInterruptStage)
	 TI_CC_EnableInterrupt();

  TI_CC_Strobe(TI_CCxxx0_SFRX);
  TI_CC_Strobe(TI_CCxxx0_SFTX);
  TI_CC_Strobe(TI_CCxxx0_SRX);      // Initialize CCxxxx in RX mode.

  return true;
}

//-----------------------------------------------------------------------------
//  e_RxStatus RfReceivePacket(uint8_t *rxBuffer)
//
//  DESCRIPTION:
//  Receives a packet of variable length (first byte in the packet must be the
//  length byte).  The packet length should not exceed the RXFIFO size.  To use
//  this function, APPEND_STATUS in the PKTCTRL1 register must be enabled.  It
//  is assumed that the function is called after it is known that a packet has
//  been received; for example, in response to GDO0 going low when it is
//  configured to output packet reception status.
//
//  The RXBYTES register is first read to ensure there are bytes in the FIFO.
//  This is done because the GDO signal will go high even if the FIFO is flushed
//  due to address filtering, CRC filtering, or packet length filtering.
//
//  ARGUMENTS:
//      uint8_t *rxBuffer
//          Pointer to the buffer where the incoming data should be stored
//
//  RETURN VALUE:
//      uint8_t
//          RX_STATUS_SUCCESS	:  CRC OK
//          RX_STATUS_CRC_ERROR	:  CRC NOT OK (or no pkt was put in the
//											 	RXFIFO due to filtering)
//			RX_STATUS_FAILED	:  False trigger
//-----------------------------------------------------------------------------
e_RxStatus RfReceivePacket(uint8_t *rxBuffer)
{
  if(TI_CC_IsCRCOK())
  {
	  if ((RfReadReg(TI_CCxxx0_RXBYTES) & TI_CCxxx0_NUM_RXBYTES))
	  {
		// Use the appropriate library function to read the first byte in the
		// RX FIFO, which is the length of the packet (the total remaining bytes
		// in this packet after reading this byte).
		// Hint:  how many bytes are being retrieved?  One or multiple?
		rxBuffer[0] = RfReadReg(TI_CCxxx0_RXFIFO);

		// Use the appropriate library function to read the rest of the packet into
		// rxBuffer (i.e., read pktLen bytes out of the FIFO)
		// Hint:  how many bytes are being retrieved?  One or multiple?
		RfReadBurstReg(TI_CCxxx0_RXFIFO, &rxBuffer[1], rxBuffer[0]); // Pull data

		TI_CC_Strobe(TI_CCxxx0_SRX);      // Initialize CCxxxx in RX mode.
		return RX_STATUS_SUCCESS;
	  }
	  else
	  {
		  TI_CC_Strobe(TI_CCxxx0_SRX);      // Initialize CCxxxx in RX mode.
		  return RX_STATUS_FAILED;
	  }
  }

  RfFlushRxFifo();
  TI_CC_Strobe(TI_CCxxx0_SRX);      // Initialize CCxxxx in RX mode.
  return RX_STATUS_CRC_ERROR;
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
	bool bCurrentInterruptStage = TI_CC_GetInterruptState();
	TI_CC_DisableInterrupt();

	bool bReturn = false;

	va_list argp;
	va_start(argp, pfnDecodePacket);	// Start the varargs processing.

	uint64_t i;
	uint8_t pui8RxBuffer[TXBUFFERSIZE] = {0};
	for(i = 0; i < ui64PeriodInUs; i++)
	{
		if (TI_CC_IsInterruptPinAsserted())
		{
			TI_CC_ClearIntFlag();

			if (RfReceivePacket(pui8RxBuffer) == RX_STATUS_SUCCESS) {   // Fetch packet from CCxxxx
				// Call packet decoder
				if((*pfnDecodePacket)(pui8RxBuffer, argp))
				{
					// if decode success then terminal this process
					bReturn = true;
					break;
				}
			}
		}
		TI_CC_Wait(1); // delay for 1us
	}

	va_end(argp);	// We're finished with the varargs now.

	// Because previous action may have assert interrupt flag
    TI_CC_ClearIntFlag();
    TI_CC_ClearPending();
	if (bCurrentInterruptStage)
		TI_CC_EnableInterrupt();	// recover last interrupt state

	return bReturn;
}

bool RfTryToCaptureRfSignal(uint64_t ui64PeriodInUs,
			bool (*pfnHandler)(va_list argp), ...)
{
	bool bCurrentInterruptStage = TI_CC_GetInterruptState();
	TI_CC_DisableInterrupt();

	bool bReturn = false;

	va_list argp;
	va_start(argp, pfnHandler);	// Start the varargs processing.

	uint64_t i;
	for(i = 0; i < ui64PeriodInUs; i++)
	{
		if (TI_CC_IsInterruptPinAsserted())
		{
			TI_CC_ClearIntFlag();

			// Call handler
			if((*pfnHandler)(argp))
			{
				// if decode success then terminal this process
				bReturn = true;
				break;
			}
		}
		TI_CC_Wait(1); // delay for 1us
	}

	va_end(argp);	// We're finished with the varargs now.

	// Because previous action may have assert interrupt flag
    TI_CC_ClearIntFlag();
    TI_CC_ClearPending();
	if (bCurrentInterruptStage)
		TI_CC_EnableInterrupt();	// recover last interrupt state

	return bReturn;
}

//----------------------------------------------------------------------------
//  void RfPowerupCSnSequence(void)
//
//  DESCRIPTION:
// 	Init sequeces reset signal in CSn pin
//  NOTE: This is optional function only use for CCxxx0 wireless module
//----------------------------------------------------------------------------
void RfPowerupCSnSequence(void)
{
  TI_CC_SetCSN();
  TI_CC_Wait(200);
  TI_CC_ClearCSN();
  TI_CC_Wait(200);
  TI_CC_SetCSN();
  TI_CC_Wait(300);
}

//----------------------------------------------------------------------------
//  void RfResetChip(void)
//
//  DESCRIPTION:
//  Send RES strobe to reset CCxxx0, RFPowerupCSnSequence must be call before
//  calling this function at the first powerup CCxxx0 state.
//----------------------------------------------------------------------------
void RfResetChip(void)
{
  TI_CC_ClearCSN();     				 				// /CS enable

  TI_CC_WaitForCCxxxxReady();

  TI_CC_SendAndGetData(TI_CCxxx0_SRES);                 // Send strobe

  TI_CC_SetCSN();       								// /CS disable
}

//----------------------------------------------------------------------------
//  uint8_t RfReadStatus(void)
//
//  DESCRIPTION:
//  Special read function for reading status registers.  Reads status register
//  at register "addr" and returns the value read.
//----------------------------------------------------------------------------
uint8_t RfReadStatus(void)
{
	return RfReadReg(TI_CCxxx0_PKTSTATUS);
}

//----------------------------------------------------------------------------
//  uint8_t RfReadReg(uint8_t addr)
//
//  DESCRIPTION:
//  Reads a single configuration register at address "addr" and returns the
//  value read.
//----------------------------------------------------------------------------
uint8_t RfReadReg(uint8_t addr)
{
  uint8_t x;

  TI_CC_ClearCSN();     				 				// /CS enable

  TI_CC_WaitForCCxxxxReady();

  TI_CC_SendAndGetData(addr | TI_CCxxx0_READ_SINGLE); 	// Send address

  x = TI_CC_SendAndGetData(0);							// Get data

  TI_CC_SetCSN();       								// /CS disable

  return x;
}

//----------------------------------------------------------------------------
//  void RfReadBurstReg(uint8_t addr, uint8_t *buffer, uint8_t count)
//
//  DESCRIPTION:
//  Reads multiple configuration registers, the first register being at address
//  "addr".  Values read are deposited sequentially starting at address
//  "buffer", until "count" registers have been read.
//----------------------------------------------------------------------------
void RfReadBurstReg(uint8_t addr, uint8_t *buffer, uint8_t count)
{
  unsigned int i;

  TI_CC_ClearCSN();     				 				// /CS enable

  TI_CC_WaitForCCxxxxReady();

  TI_CC_SendAndGetData(addr | TI_CCxxx0_READ_BURST);  	// Send address

  for (i = 0; i < count; i++)
  {
	buffer[i] = TI_CC_SendAndGetData(0);				// Get data
  }

  TI_CC_SetCSN();       								// /CS disable
}

//----------------------------------------------------------------------------
//  void RfWriteReg(uint8_t addr, uint8_t value)
//
//  DESCRIPTION:
//  Writes "value" to a single configuration register at address "addr".
//----------------------------------------------------------------------------
void RfWriteReg(uint8_t addr, uint8_t value)
{
	TI_CC_ClearCSN();     				 	// /CS enable

	TI_CC_WaitForCCxxxxReady();

    TI_CC_SendAndGetData(addr);				// Send address

    TI_CC_SendAndGetData(value);            // Load data for TX after addr

    TI_CC_SetCSN();       					// /CS disable
}

//----------------------------------------------------------------------------
//  void RfWriteBurstReg(uint8_t addr, uint8_t *buffer, uint8_t count)
//
//  DESCRIPTION:
//  Writes values to multiple configuration registers, the first register being
//  at address "addr".  First data byte is at "buffer", and both addr and
//  buffer are incremented sequentially (within the CCxxxx and MSP430,
//  respectively) until "count" writes have been performed.
//----------------------------------------------------------------------------
void RfWriteBurstReg(uint8_t addr, uint8_t *buffer, uint8_t count)
{
	uint8_t i;

	TI_CC_ClearCSN();     				 				// /CS enable

	TI_CC_WaitForCCxxxxReady();

	TI_CC_SendAndGetData(addr | TI_CCxxx0_WRITE_BURST); // Send address

    for (i = 0; i < count; i++)
    {
    	TI_CC_SendAndGetData(buffer[i]);                // Send data
    }

    TI_CC_SetCSN();       								// /CS disable
}

//-----------------------------------------------------------------------------
//  void initRfModule(bool isEnableInt)
//
//  DESCRIPTION:
//  configure SSI peripheral and initialize CC2500
//  set to first channel in freqTab, enable RxMode before return
//
//  ARGUMENTS:
//		bool isEnableInt
//			True for enable interrupt pin at GPO0, false for disable
//-----------------------------------------------------------------------------
void initRfModule(bool isEnableInt)
{
  uint8_t paTable[] = { TI_CCxxx0_OUTPUT_POWER_0DB }; // Table 31 - Page 47 of 89
  uint8_t paTableLen = 1;

  TI_CC_Setup();           			// Initialize SPI port with interrupt enable

  RfPowerupCSnSequence();
  RfResetChip();               		// Reset CCxxxx RF device
  RfWriteSettings();           		// Write RF settings to config reg
  RfWriteBurstReg(TI_CCxxx0_PATABLE, // Write the PATABLE
                         paTable, paTableLen);

  char commonAddress = 0;			// Set my own device address
  while (commonAddress != TI_CCxxx0_COMMON_ADDRESS) {
	RfWriteReg(TI_CCxxx0_ADDR, TI_CCxxx0_COMMON_ADDRESS);
	commonAddress = RfReadReg(TI_CCxxx0_ADDR);
  }

  RfSetChannel(0);		  			// Set to default channel

  TI_CC_ConfigIRQPin(isEnableInt);		// Configre interrupt GDO0

  TI_CC_Strobe(TI_CCxxx0_SIDLE);	// Idle mode
  TI_CC_Strobe(TI_CCxxx0_SFRX);		// Flush RX FIFO
  TI_CC_Strobe(TI_CCxxx0_SFTX);		// Flush TX FIFO
  TI_CC_Strobe(TI_CCxxx0_SRX);      // Initialize CCxxxx in RX mode.

  TI_CC_Wait(260);	// Wait 260us for stable initilization
}

//-----------------------------------------------------------------------------
//  void RfSetChannel(uint8_t chanNum)
//
//  DESCRIPTION:
//  set CC2500 channel
//
//  ARGUMENTS:
//      int chanNum
//          Pointer to the channel in freqTab
//-----------------------------------------------------------------------------
void RfSetChannel(uint8_t chanNum)
{
  uint8_t freqTab[RFNUMCHANS][3] = {
		  {0x13, 0xBB, 0x5D},  // 2.4370 GHz (default)
		  {0xB1, 0x53, 0x5C},  // 2.4005 GHz
		  {0x13, 0x3B, 0x5D},  // 2.4240 GHz
		  {0xB1, 0x93, 0x5D},  // 2.4329 GHz
		  {0x76, 0x62, 0x5F},  // 2.4800 GHz
  	  	  };

  uint8_t data, freq2, freq1, freq0;


  freq0 = freqTab[chanNum][0];
  freq1 = freqTab[chanNum][1];
  freq2 = freqTab[chanNum][2];


  data = 0;
  while (data != freq2) {
    RfWriteReg(TI_CCxxx0_FREQ2, freq2);
    data = RfReadReg(TI_CCxxx0_FREQ2);
  }
  data = 0;
  while (data != freq1) {
    RfWriteReg(TI_CCxxx0_FREQ1, freq1);
    data = RfReadReg(TI_CCxxx0_FREQ1);
  }
  data = 0;
  while (data != freq0) {
    RfWriteReg(TI_CCxxx0_FREQ0, freq0);
    data = RfReadReg(TI_CCxxx0_FREQ0);
  }
}

//-----------------------------------------------------------------------------
//  void RfSetIdleMode(void)
//
//  DESCRIPTION:
//	forces CC2500 to idle mode
//-----------------------------------------------------------------------------
void RfSetIdleMode(void)
{
	TI_CC_Strobe(TI_CCxxx0_SIDLE);         // Initialize CCxxxx to IDLE mode.
}

//-----------------------------------------------------------------------------
//  void RfSetRxMode(void)
//
//  DESCRIPTION:
//	forces CC2500 to Rx mode or WOR mode corresponding to rxMode variables
//-----------------------------------------------------------------------------
void RfSetRxMode(void)
{
  TI_CC_Strobe(TI_CCxxx0_SIDLE);         // Initialize CCxxxx to IDLE mode.
  TI_CC_Strobe(TI_CCxxx0_SRX);           // Initialize CCxxxx in RX mode.
}

//-----------------------------------------------------------------------------
//  void RfFlushTxFifo(void)
//
//  DESCRIPTION:
//	send Flush TX FIFO strobe
//-----------------------------------------------------------------------------
void RfFlushTxFifo(void)
{
	TI_CC_Strobe(TI_CCxxx0_SFTX);           // Flush the TX FIFO
}

//-----------------------------------------------------------------------------
//  void RfFlushRxFifo(void)
//
//  DESCRIPTION:
//	send Flush RX FIFO strobe
//-----------------------------------------------------------------------------
void RfFlushRxFifo(void)
{
	TI_CC_Strobe(TI_CCxxx0_SFRX);          // Flush the RX FIFO
}



