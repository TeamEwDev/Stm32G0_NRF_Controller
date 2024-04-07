/**
  ******************************************************************************
  * @file    nrf24.h
  * @brief   Header file for NRF24 module communication
  * @author  EW Dev
  ******************************************************************************
  */

#ifndef NRF24_H
#define NRF24_H

#include <stdint.h>
#include "nRF24L01.h"
#include "stm32g0xx_hal.h"
#include "main.h"

extern SPI_HandleTypeDef hspi1;

#define NRF24_CSN_PIN_SET(state)  HAL_GPIO_WritePin(NRF24_CSN_GPIO_Port, NRF24_CSN_Pin, state)
#define NRF24_CE_PIN_SET(state)   HAL_GPIO_WritePin(NRF24_SPI1_CS_GPIO_Port, NRF24_SPI1_CS_Pin, state)

#define HAL_SPI_TRANSMIT_RECEIVE(txData, rxData, len) HAL_SPI_TransmitReceive(&hspi1, txData, rxData, len, 1000)
#define HAL_SPI_TRANSMIT(txData, len) HAL_SPI_Transmit(&hspi1, txData, len, 1000)

#define LOW 0
#define HIGH 1

#define NRF24_ADDR_LEN 5
#define NRF24_CONFIG ((1 << EN_CRC) | (0 << CRCO)) //| (0 << MASK_MAX_RT) | (1 << MASK_TX_DS) | (0 << MASK_RX_DR)

#define NRF24_TRANSMISSION_OK 0
#define NRF24_MESSAGE_LOST 1


/* Initialization and configuration functions */
void NRF24_Init();
void NRF24_Rx_Address(uint8_t *adr);
void NRF24_Tx_Address(uint8_t *adr);
void NRF24_Config(uint8_t channel, uint8_t pay_length);

/* State check functions */
uint8_t NRF24_Data_Ready();
uint8_t NRF24_Is_Sending();
uint8_t NRF24_Get_Status();
uint8_t NRF24_RxFifo_Empty();

/* Core TX / RX functions */
void NRF24_Send(uint8_t *value);
void NRF24_Get_Data(uint8_t *data);

/* Functions for dynamic length mode */
uint8_t NRF24_Payload_Length();

/* Post-transmission analysis functions */
uint8_t NRF24_Last_Message_Status();
uint8_t NRF24_Retransmission_Count();

/* Power management functions */
void NRF24_Power_Up_Rx();
void NRF24_Power_Up_Tx();
void NRF24_Power_Down();
void NRF24_IRQ_Config(void);

/* Low-level interface functions */
void NRF24_Transmit_Sync(uint8_t *dataout, uint8_t len);
void NRF24_Transfer_Sync(uint8_t *dataout, uint8_t *datain, uint8_t len);
void NRF24_Config_Register(uint8_t reg, uint8_t value);
void NRF24_Read_Register(uint8_t reg, uint8_t *value, uint8_t len);
void NRF24_Write_Register(uint8_t reg, uint8_t *value, uint8_t len);

#endif /* NRF24_H */
