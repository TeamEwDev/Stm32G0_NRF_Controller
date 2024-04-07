/**
  ******************************************************************************
  * @file    nrf24.c
  * @brief   Source file for NRF24 module communication
  * @author  EW Dev
  ******************************************************************************
  */


#include "nrf24.h"

uint8_t payload_len; /**< Length of payload for NRF24 */

/**
 * @brief Initialize NRF24 module by setting hardware pins
 */
void NRF24_Init()
{
    NRF24_CE_PIN_SET(LOW);  // Set CE pin LOW
    NRF24_CSN_PIN_SET(HIGH); // Set CSN pin HIGH
}

/**
 * @brief Configure NRF24 module with specified channel and payload length
 * @param channel RF channel to use
 * @param pay_length Length of payload
 */
void NRF24_Config(uint8_t channel, uint8_t pay_length)
{
    payload_len = pay_length; // Set static payload length

    // Set RF channel
    NRF24_Config_Register(RF_CH, channel);

    // Set length of incoming payload for data pipe 1
    NRF24_Config_Register(RX_PW_P0, 0x00); // Auto-ACK pipe ...
    NRF24_Config_Register(RX_PW_P1, payload_len);
    NRF24_Config_Register(RX_PW_P2, 0x00); // Pipe not used
    NRF24_Config_Register(RX_PW_P3, 0x00); // Pipe not used
    NRF24_Config_Register(RX_PW_P4, 0x00); // Pipe not used
    NRF24_Config_Register(RX_PW_P5, 0x00); // Pipe not used


    // Configure RF setup (1 Mbps, 0 dBm TX power)
    NRF24_Config_Register(RF_SETUP, (0 << RF_DR) | ((0x03) << RF_PWR));

    // Enable CRC, 1-byte CRC length, and auto acknowledgment on data pipe 0 and 1
    NRF24_Config_Register(CONFIG, NRF24_CONFIG);

    // Auto Acknowledgment
    NRF24_Config_Register(EN_AA, (1 << ENAA_P0) | (1 << ENAA_P1) | (0 << ENAA_P2) | (0 << ENAA_P3) | (0 << ENAA_P4) |
                          (0 << ENAA_P5));
    // Enable RX addresses
    NRF24_Config_Register(EN_RXADDR, (1 << ERX_P0) | (1 << ERX_P1) | (0 << ERX_P2) | (0 << ERX_P3) | (0 << ERX_P4) |
                          (0 << ERX_P5));

    // Configure auto retransmit delay (1000 us) and up to 15 retransmit trials
    NRF24_Config_Register(SETUP_RETR, (0x04 << ARD) | (0x0F << ARC));

    // Dynamic length configurations: No dynamic length
    NRF24_Config_Register(DYNPD, (0 << DPL_P0) | (0 << DPL_P1) | (0 << DPL_P2) | (0 << DPL_P3) | (0 << DPL_P4) |
                          (0 << DPL_P5));

    // Start listening in RX mode
    NRF24_Power_Up_Rx();
}

/**
 * @brief Set receive address for NRF24 module
 * @param addr Pointer to array containing address data
 */
void NRF24_Rx_Address(uint8_t *addr)
{
    NRF24_CE_PIN_SET(LOW); // Set CE pin LOW
    NRF24_Write_Register(RX_ADDR_P1, addr, NRF24_ADDR_LEN); // Write address to data pipe 1
    NRF24_CE_PIN_SET(HIGH); // Set CE pin HIGH to apply address
}

/**
 * @brief Set transmit address for NRF24 module
 * @param addr Pointer to array containing address data
 */
void NRF24_Tx_Address(uint8_t *addr)
{
    // Set RX_ADDR_P0 to the transmit address for auto acknowledgment
    NRF24_Write_Register(RX_ADDR_P0, addr, NRF24_ADDR_LEN);
    // Set TX_ADDR to the transmit address
    NRF24_Write_Register(TX_ADDR, addr, NRF24_ADDR_LEN);
}

/**
 * @brief Check if data is ready for reading from NRF24 module
 * @return 1 if data is ready, 0 otherwise
 */
uint8_t NRF24_Data_Ready()
{
    uint8_t status = NRF24_Get_Status();
    // We can short circuit on RX_DR, but if it's not set, we still need
    // to check the FIFO for any pending packets
    if (status & (1 << RX_DR))
    {
        return 1;
    }

    return !NRF24_RxFifo_Empty();;
}

/* Checks if receive FIFO is empty or not */
uint8_t NRF24_RxFifo_Empty()
{
    uint8_t fifoStatus;
    NRF24_Read_Register(FIFO_STATUS, &fifoStatus, 1);
    return (fifoStatus & (1 << RX_EMPTY));
}

/**
 * @brief Get the length of data waiting in the receive FIFO
 * @return Length of waiting payload
 */
uint8_t NRF24_Payload_Length()
{
    uint8_t status, txData = R_RX_PL_WID;
    NRF24_CSN_PIN_SET(LOW);
    HAL_SPI_TRANSMIT_RECEIVE(&txData, &status, 1);
    NRF24_CSN_PIN_SET(HIGH);
    return status;
}

/**
 * @brief Read received payload data from NRF24 module
 * @param data Pointer to array to store received data
 */
void NRF24_Get_Data(uint8_t *data)
{
    uint8_t txData = R_RX_PAYLOAD;
    /* Pull down chip select */
    NRF24_CSN_PIN_SET(LOW);
    /* Send cmd to read rx payload */
    HAL_SPI_TRANSMIT(&txData, 1);
    /* Read payload */
    NRF24_Transfer_Sync(data, data, payload_len);
    /* Pull up chip select */
    NRF24_CSN_PIN_SET(HIGH);
    /* Reset status register */
    NRF24_Config_Register(STATUS, (1 << RX_DR));
}

/* Returns the number of retransmissions occured for the last message */
uint8_t NRF24_Retransmission_Count()
{
    uint8_t rv;
    NRF24_Read_Register(OBSERVE_TX, &rv, 1);
    rv = rv & 0x0F;
    return rv;
}
/**
 * @brief Sends a data package to the default address
 * @details This function sends a data package to the default address configured on the NRF24 module.
 *          Ensure to send the correct amount of bytes as configured as payload on the receiver.
 * @param value Pointer to the data array to be sent
 */
void NRF24_Send(uint8_t *value)
{
    uint8_t txData = FLUSH_TX;

    // Go to Standby-I mode first
    NRF24_CE_PIN_SET(LOW);

    // Set to transmitter mode and power up if needed
    NRF24_Power_Up_Tx();

#if 1
    // Flush transmit FIFO
    NRF24_CSN_PIN_SET(LOW);
    HAL_SPI_TRANSMIT(&txData, 1);
    NRF24_CSN_PIN_SET(HIGH);
#endif

    // Write payload command
    NRF24_CSN_PIN_SET(LOW);
    txData = W_TX_PAYLOAD;
    HAL_SPI_TRANSMIT(&txData, 1);

    // Write payload data
    NRF24_Transmit_Sync(value, payload_len);

    // Complete the transmission
    NRF24_CSN_PIN_SET(HIGH);
    NRF24_CE_PIN_SET(HIGH);
}

/**
 * @brief Checks if NRF24 module is currently sending data
 * @details This function checks if the NRF24 module is currently sending data.
 * @return 1 if the module is sending data, 0 otherwise
 */
uint8_t NRF24_Is_Sending()
{
    uint8_t status = NRF24_Get_Status();

    // Check if sending successful (TX_DS) or max retries exceeded (MAX_RT)
    if ((status & ((1 << TX_DS) | (1 << MAX_RT))))
    {
        return 0; // Not sending
    }

    return 1; // Sending
}

/**
 * @brief Gets the status register value from NRF24 module
 * @details This function retrieves the status register value from the NRF24 module.
 * @return Status register value
 */
uint8_t NRF24_Get_Status()
{
    uint8_t rv, txData = NOP;
    NRF24_CSN_PIN_SET(LOW);
    HAL_SPI_TRANSMIT_RECEIVE(&txData, &rv, 1);
    NRF24_CSN_PIN_SET(HIGH);
    return rv;
}

/**
 * @brief Checks the status of the last message sent by NRF24 module
 * @details This function checks the status of the last message sent by the NRF24 module.
 *          It determines whether the transmission was successful, failed due to maximum retries,
 *          or is still in progress.
 * @return Transmission status code:
 *         - NRF24_TRANSMISSION_OK if transmission was successful
 *         - NRF24_MESSAGE_LOST if maximum retransmission count is reached
 *         - 0xFF if transmission status is unknown
 */
uint8_t NRF24_Last_Message_Status()
{
    uint8_t rv = NRF24_Get_Status();

    // Check transmission status
    if ((rv & (1 << TX_DS)))
    {
        return NRF24_TRANSMISSION_OK; // Transmission successful
    }
    else if ((rv & (1 << MAX_RT)))
    {
        return NRF24_MESSAGE_LOST; // Maximum retransmission count reached
    }
    else
    {
        return 0xFF; // Unknown status
    }
}

/**
 * @brief Powers up NRF24 module in RX mode
 * @details This function powers up the NRF24 module in receive (RX) mode.
 *          It flushes the RX FIFO, configures the status register, and enables RX mode.
 */
void NRF24_Power_Up_Rx()
{
    uint8_t txData = FLUSH_RX;
    NRF24_CSN_PIN_SET(LOW);
    HAL_SPI_TRANSMIT(&txData, 1);
    NRF24_CSN_PIN_SET(HIGH);

    // Configure status register for RX mode
    NRF24_Config_Register(STATUS, (1 << RX_DR) | (1 << TX_DS) | (1 << MAX_RT));

    NRF24_CE_PIN_SET(LOW);
    // Configure CONFIG register for RX mode with power up
    NRF24_Config_Register(CONFIG, NRF24_CONFIG | ((1 << PWR_UP) | (1 << PRIM_RX)));
    NRF24_CE_PIN_SET(HIGH); // Set CE pin HIGH to start listening
}

/**
 * @brief Powers up NRF24 module in TX mode
 * @details This function powers up the NRF24 module in transmit (TX) mode.
 *          It configures the status register for TX mode without enabling RX mode.
 */
void NRF24_Power_Up_Tx()
{
    // Configure status register for TX mode
    NRF24_Config_Register(STATUS, (1 << RX_DR) | (1 << TX_DS) | (1 << MAX_RT));

    // Configure CONFIG register for TX mode with power up
    NRF24_Config_Register(CONFIG, NRF24_CONFIG | ((1 << PWR_UP) | (0 << PRIM_RX)));
}

/**
 * @brief Powers down NRF24 module
 * @details This function powers down the NRF24 module by setting CE pin LOW and configuring the CONFIG register.
 */
void NRF24_Power_Down()
{
    NRF24_CE_PIN_SET(LOW);
    NRF24_Config_Register(CONFIG, NRF24_CONFIG); // Reset CONFIG register
}

/**
 * @brief Performs SPI transfer synchronously for data I/O
 * @details This function performs synchronous SPI transfer for sending and receiving data.
 * @param dataout Pointer to the data to be transmitted
 * @param datain Pointer to store the received data
 * @param len Number of bytes to transmit and receive
 */
void NRF24_Transfer_Sync(uint8_t *dataout, uint8_t *datain, uint8_t len)
{
    HAL_SPI_TRANSMIT_RECEIVE(dataout, datain, len);
}

/**
 * @brief Sends multiple bytes over SPI
 * @details This function sends multiple bytes of data over SPI.
 * @param dataout Pointer to the data array to be transmitted
 * @param len Number of bytes to transmit
 */
void NRF24_Transmit_Sync(uint8_t *dataout, uint8_t len)
{
    HAL_SPI_TRANSMIT(dataout, len);
}

/** Clocks a single byte into the specified nrf24 register.
 *
 * @param reg   Register address to write to.
 * @param value Value to write into the register.
 */
void NRF24_Config_Register(uint8_t reg, uint8_t value)
{
    uint8_t txData = W_REGISTER | (REGISTER_MASK & reg);  // Construct write command
    NRF24_CSN_PIN_SET(LOW);                                // Select the device
    HAL_SPI_TRANSMIT(&txData, 1);                          // Send register write command
    HAL_SPI_TRANSMIT(&value, 1);                           // Send register value
    NRF24_CSN_PIN_SET(HIGH);                               // Deselect the device
}

/** Reads a single register from the nrf24 device.
 *
 * @param reg   Register address to read from.
 * @param value Pointer to where the register value will be stored.
 * @param len   Length of data to read (usually 1 byte).
 */
void NRF24_Read_Register(uint8_t reg, uint8_t *value, uint8_t len)
{
    uint8_t txData = R_REGISTER | (REGISTER_MASK & reg);   // Construct read command
    NRF24_CSN_PIN_SET(LOW);                                // Select the device
    HAL_SPI_TRANSMIT(&txData, 1);                          // Send register read command
    NRF24_Transfer_Sync(value, value, len);                // Receive register value
    NRF24_CSN_PIN_SET(HIGH);                               // Deselect the device
}

/**
 * @brief Writes to a single register of NRF24 module
 * @details This function writes data to a single register of the NRF24 module.
 * @param reg Register address to write to
 * @param value Pointer to the data array containing the value(s) to be written
 * @param len Number of bytes to write
 */
void NRF24_Write_Register(uint8_t reg, uint8_t *value, uint8_t len)
{
    uint8_t txData = W_REGISTER | (REGISTER_MASK & reg);
    NRF24_CSN_PIN_SET(LOW);
    HAL_SPI_TRANSMIT(&txData, 1);
    NRF24_Transmit_Sync(value, len);
    NRF24_CSN_PIN_SET(HIGH);
}

/**
 * @brief Configures NRF24L01+ module for IRQ (Interrupt Request) usage.
 *
 * This function initializes the NRF24L01+ module to enable IRQs for specific events
 * such as data ready (RX_DR) and successful transmission (TX_DS). It configures
 * relevant registers on the NRF24L01+ module via SPI communication.
 *
 * @note This function assumes that the NRF24L01+ module has been properly initialized
 *       and SPI communication is set up.
 *
 * @return None
 *
 * @par Example:
 * @code
 * // Configure IRQ functionality for NRF24L01+ module
 * NRF24_IRQ_Config();
 * @endcode
 */
void NRF24_IRQ_Config(void)
{
    uint8_t config = 0;

    // Configure the CONFIG register to enable IRQs for data ready (RX_DR) and TX_DS
    config |= (1 << MASK_RX_DR); // Enable IRQ for data ready (RX_DR)
    config |= (1 << MASK_TX_DS); // Enable IRQ for successful transmission

    // Set up CONFIG register
    NRF24_Write_Register(CONFIG, &config, 1);
    // Clear any pending interrupts in STATUS register
    NRF24_Write_Register(STATUS, &config, 1);
}
