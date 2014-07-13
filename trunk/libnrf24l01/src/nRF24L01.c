//  ***  modified by VyLong  *** //

#include "libnrf24l01/inc/TM4C123_nRF24L01.h"
#include "libnrf24l01/inc/nRF24L01.h"

// Configuration parameters used to set-up the RF configuration
static unsigned char crcConfig;
static unsigned char addressWidth;

// Status variable updated every time SPI I/O is performed
static unsigned char status;

// Used to track which features have been enabled
static char features;

void RF24_init(const RF24_InitTypeDef* InitRf24)
{
    initSpiForRF(InitRf24->InterruptEnable);
    setRfCSN();
    clearRfCE();

    // Wait 100ms for RF transceiver to initialize.
    unsigned char c = 20;
    for (; c; c--)
    {
       rfDelayLoop(DELAY_CYCLES_5MS);
    }

    RF24_clearIrqFlag(RF24_IRQ_MASK);

    RF24_PIPE_closeAll();

    RF24_RETRANS_setDelay(InitRf24->RetransmitDelay);
    RF24_RETRANS_setCount(InitRf24->RetransmitCount);
    RF24_setSpeedAndPower(InitRf24->Speed|InitRf24->Power|InitRf24->LNAGainEnable);
    RF24_setChannel(InitRf24->Channel);
    RF24_setAddressWidth(InitRf24->AddressWidth);

    features = 0x00;
    RF24_enableFeatures(InitRf24->Features);

    RF24_TX_flush();
    RF24_RX_flush();

    crcConfig = InitRf24->CrcState | InitRf24->CrcBytes;
    addressWidth = InitRf24->AddressWidth + 2;
}

void RF24_enableFeatures (unsigned char enFeature)
{
    if ((features & enFeature) != enFeature)
    {
      features |= enFeature;
      features &= 0x07;
      RF24_writeRegister(RF24_REG_FEATURE, features);
    }
}

void RF24_disableFeatures (unsigned char disfeature)
{
    if (features & disfeature)
    {
        features &= ~disfeature;
        RF24_writeRegister(RF24_REG_FEATURE, features);
    }
}

unsigned char RF24_getLostPackets ()
{
    return (RF24_readRegister(RF24_REG_OBSERVE_TX) >> 4) & 0x0F;
}

unsigned char RF24_isAlive ()
{
    unsigned char aw;

    aw = RF24_readRegister(RF24_REG_SETUP_AW);
    return((aw & 0xFC) == 0x00 && (aw & 0x03) != 0x00);
}

inline void RF24_setConfigureRegister (unsigned char newConfigure)
{
  RF24_writeRegister(RF24_REG_CONFIG, (newConfigure & RF24_REG_CONFIG_MASK));
}

inline unsigned char RF24_getConfigureRegister ()
{
  return RF24_readRegister(RF24_REG_CONFIG);
}

void RF24_setSpeedAndPower (unsigned char rfSpeedAndPower)
{
   rfSpeedAndPower &= 0x0F;
   RF24_writeRegister(RF24_REG_RF_SETUP, rfSpeedAndPower);
}

void RF24_setChannel (unsigned char rfChannel)
{
    RF24_writeRegister(RF24_REG_RF_CH, (rfChannel & 0x7F));
}

void RF24_setAddressWidth (unsigned char rfAddressWidth)
{
    rfAddressWidth &= 0x03;
    RF24_writeRegister(RF24_REG_SETUP_AW, rfAddressWidth);
}

unsigned char RF24_getCurrentState ()
{
    unsigned char config;

    if (!RF24_isAlive())
        return RF24_STATE_NOTPRESENT;

    config = RF24_readRegister(RF24_REG_CONFIG);
    if ((config & RF24_PWR_UP) == RF24_PWR_DOWN)
        return RF24_STATE_POWERDOWN;

    if (getCEState() == 0)
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

void RF24_startPowerDown ()
{
    clearRfCE();
    RF24_setConfigureRegister(crcConfig | RF24_PWR_DOWN | RF24_PRIM_RX);
}

void RF24_startStandby ()
{
    unsigned char config = RF24_readRegister(RF24_REG_CONFIG);

    clearRfCE();

    if ((config & RF24_PWR_UP) == RF24_PWR_DOWN)
    {
        if (config & RF24_PRIM_RX)
          RF24_setConfigureRegister(crcConfig | RF24_PWR_UP | RF24_PRIM_RX);
        else
          RF24_setConfigureRegister(crcConfig | RF24_PWR_UP | RF24_PRIM_TX);

        rfDelayLoop(DELAY_CYCLES_5MS);
    }
}

unsigned char RF24_scan ()
{
    int testcount = 1023;
    unsigned int rpdcount = 0;
    unsigned char last_state;

    last_state = RF24_getCurrentState();
    if (last_state != RF24_STATE_PRX)
            RF24_RX_activate();

    for (; testcount > 0; testcount--) {
            if (RF24_readRegister(RF24_REG_RPD))
                    rpdcount++;
            rfDelayLoop(DELAY_CYCLES_130US);
            RF24_RX_flush();
            RF24_clearIrqFlag(RF24_IRQ_RX);
    }

    if (last_state != RF24_STATE_PRX)
            RF24_startStandby();
    return( (unsigned char) (rpdcount >> 2) );
}

unsigned char RF24_getIrqFlag (unsigned char rfIrqFlag)
{
    clearRfCSN();
    status = SPI_sendAndGetData(RF24_COMMAND_NOP);
    setRfCSN();
    status &= RF24_IRQ_MASK;
    status &= rfIrqFlag;
    return status;
}

void RF24_clearIrqFlag (unsigned char irqflag)
{
    clearRfCSN();
    status = SPI_sendAndGetData(RF24_REG_STATUS | RF24_COMMAND_W_REGISTER);
    status &= 0x0F;
    irqflag |= status;
    SPI_sendAndGetData(irqflag);
    setRfCSN();
}

unsigned char RF24_readRegister(unsigned char addr)
{
    unsigned char i;
    clearRfCSN();
    status = SPI_sendAndGetData((addr & RF24_REG_MASK) | RF24_COMMAND_R_REGISTER);
    i = SPI_sendAndGetData(RF24_COMMAND_NOP);
    setRfCSN();
    return i;
}

inline void RF24_writeRegister(unsigned char addr, unsigned char data)
{
    clearRfCSN();
    status = SPI_sendAndGetData((addr & RF24_REG_MASK) | RF24_COMMAND_W_REGISTER);
    SPI_sendAndGetData(data);
    setRfCSN();
}

void RF24_TX_setAddress(unsigned char *addr)
{
    signed char i;
    clearRfCSN();
    status = SPI_sendAndGetData(RF24_REG_TX_ADDR | RF24_COMMAND_W_REGISTER);
    for (i = 0; i < addressWidth; i++)
    {
        SPI_sendAndGetData(addr[i]);
    }
    setRfCSN();
}

void RF24_TX_writePayloadAck(unsigned char len, unsigned char *data)
{
    unsigned char i=0;
    clearRfCSN();
    status = SPI_sendAndGetData(RF24_COMMAND_W_TX_PAYLOAD);
    for (i = 0; i < len; i++)
    {
        SPI_sendAndGetData(data[i]);
    }
    setRfCSN();
}

void RF24_TX_writePayloadNoAck(unsigned char len, unsigned char *data)
{
    unsigned char i=0;
    if ( !(features & RF24_FEATURE_EN_NO_ACK_COMMAND) )
            return;
    clearRfCSN();
    status = SPI_sendAndGetData(RF24_COMMAND_W_TX_PAYLOAD_NOACK);
    for (i = 0; i < len; i++)
    {
        SPI_sendAndGetData(data[i]);
    }
    setRfCSN();
}

void RF24_TX_flush()
{
    clearRfCSN();
    status = SPI_sendAndGetData(RF24_COMMAND_FLUSH_TX);
    setRfCSN();
}

void RF24_TX_reuseLastPayload()
{
    clearRfCSN();
    status = SPI_sendAndGetData(RF24_COMMAND_REUSE_TX_PL);
    setRfCSN();
}

void RF24_TX_activate()
{
    RF24_startStandby();
    RF24_TX_flush();
    RF24_setConfigureRegister(crcConfig | RF24_PWR_UP | RF24_PRIM_TX);
    RF24_clearIrqFlag(RF24_IRQ_TX | RF24_IRQ_MAX_RETRANS);
    clearRfCE();
}

void RF24_TX_pulseTransmit()
{
  setRfCE();
  rfDelayLoop(DELAY_CYCLES_15US);
  clearRfCE();
}

void RF24_RX_setAddress(unsigned char pipe, unsigned char *addr)
{
    signed char i;
    if (pipe > 5)
      return;

    clearRfCSN();
    status = SPI_sendAndGetData((RF24_REG_RX_ADDR_P0 + pipe) | RF24_COMMAND_W_REGISTER);
    if (pipe > 1)
    {
        // Pipes 2-5 differ from pipe1's addr only in the Least Significant Byte.
        SPI_sendAndGetData(addr[0]);
    }
    else
    {
        for (i = 0; i < addressWidth; i++)
        {
            SPI_sendAndGetData(addr[i]);
        }
    }
    setRfCSN();
}

unsigned char RF24_RX_getPayloadWidth()
{
    unsigned char i;
    clearRfCSN();
    status = SPI_sendAndGetData(RF24_COMMAND_R_RX_PL_WID);
    i = SPI_sendAndGetData(RF24_COMMAND_NOP);
    setRfCSN();
    return i;
}

unsigned char RF24_RX_getPayloadData(unsigned char len, unsigned char *data)
{
    unsigned char i;
    clearRfCSN();
    status = SPI_sendAndGetData(RF24_COMMAND_R_RX_PAYLOAD);
    for (i = 0; i < len; i++)
    {
      data[i] = SPI_sendAndGetData(RF24_COMMAND_NOP);
    }
    setRfCSN();

    // The RX pipe this data belongs to is stored in STATUS
    return ((status & 0x0E) >> 1);
}

void RF24_RX_flush()
{
    clearRfCSN();
    status = SPI_sendAndGetData(RF24_COMMAND_FLUSH_RX);
    setRfCSN();
}

void RF24_RX_activate ()
{
    RF24_startStandby();
    RF24_RX_flush();
    RF24_clearIrqFlag(RF24_IRQ_RX);
    RF24_setConfigureRegister(crcConfig | RF24_PWR_UP | RF24_PRIM_RX);
    setRfCE();
}

unsigned char RF24_RX_isEmpty ()
{
    unsigned char fifoStatus;

    fifoStatus = RF24_readRegister(RF24_REG_FIFO_STATUS);

    if(fifoStatus & RF24_RX_EMPTY)
    	return 1;

    return 0;
}

void RF24_RX_sendAckWithPayload (unsigned char pipe, unsigned char len, unsigned char *data)
{
    unsigned char i;

    if (pipe > 5)
            return;

    if ( !(features & RF24_FEATURE_EN_ACK_WITH_PAYLOAD) )
            return;

    clearRfCSN();
    status = SPI_sendAndGetData(RF24_COMMAND_W_ACK_PAYLOAD | pipe);
    for (i = 0; i < len; i++) {
            SPI_sendAndGetData(data[i]);
    }
    setRfCSN();
}

bool RF24_RX_carrierDetection()
{
  unsigned char CD;

  CD = RF24_readRegister(RF24_REG_CD);
  return (CD & 0x01) ;
}

void RF24_PIPE_close (unsigned char pipeid)
{
    unsigned char rxen, enaa;
    if (pipeid > 5)
      return;

    rxen = RF24_readRegister(RF24_REG_EN_RXADDR);
    enaa = RF24_readRegister(RF24_REG_EN_AA);

    rxen &= ~(1 << pipeid);
    enaa &= ~(1 << pipeid);

    RF24_writeRegister(RF24_REG_EN_RXADDR, rxen);
    RF24_writeRegister(RF24_REG_EN_AA, enaa);
}

void RF24_PIPE_closeAll ()
{
    RF24_writeRegister(RF24_REG_EN_RXADDR, 0x00);
    RF24_writeRegister(RF24_REG_EN_AA, 0x00);
    RF24_writeRegister(RF24_REG_DYNPD, 0x00);
}

void RF24_PIPE_open (unsigned char pipe, bool autoack)
{
    unsigned char rxen, enaa;

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

unsigned char RF24_PIPE_isOpen (unsigned char pipeid)
{
    unsigned char rxen;
    if (pipeid > 5)
      return 0;

    rxen = RF24_readRegister(RF24_REG_EN_RXADDR);

    return ( (1<<pipeid) == (rxen & (1<<pipeid)) );
}

void RF24_PIPE_setPacketSize (unsigned char pipe, unsigned char size)
{
    unsigned char dynpdcfg;

    dynpdcfg = RF24_readRegister(RF24_REG_DYNPD);
    if (size < 1)
    {
        // Cannot set dynamic payload if EN_DPL is disabled.
        if ( !(features & RF24_FEATURE_EN_DYNAMIC_PAYLOAD) )
              return;
        if (!( (1<<pipe) & dynpdcfg ))
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

void RF24_RETRANS_setDelay (unsigned char delay)
{
    unsigned char retransRegsiterValue;
    delay &= 0xF0;
    retransRegsiterValue = RF24_readRegister(RF24_REG_SETUP_RETR);
    retransRegsiterValue &= 0x0F;
    RF24_writeRegister(RF24_REG_SETUP_RETR, delay | retransRegsiterValue);
}

void RF24_RETRANS_setCount (unsigned char count)
{
    unsigned char retransRegsiterValue;
    count &= 0x0F;
    retransRegsiterValue = RF24_readRegister(RF24_REG_SETUP_RETR);
    retransRegsiterValue &= 0xF0;
    RF24_writeRegister(RF24_REG_SETUP_RETR, count | retransRegsiterValue);
}

unsigned char RF24_RETRANS_getLastRetransmits ()
{
    return RF24_readRegister(RF24_REG_OBSERVE_TX) & 0x0F;
}

