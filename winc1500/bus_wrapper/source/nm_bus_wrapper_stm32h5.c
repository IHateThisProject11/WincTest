/**
 *
 * \file
 *
 * \brief This module contains NMC1000 bus wrapper APIs implementation.
 *
 * Copyright (c) 2016-2017 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>     /* Included for uint_t */

#include "stm32h5xx_hal.h"
#include "stm32h5xx_hal_spi.h"

#include "bsp/include/nm_bsp.h"
#include "common/include/nm_common.h"
#include "bus_wrapper/include/nm_bus_wrapper.h"
#include "conf_winc.h"
#include "nm_bus_wrapper.h"
#include "nm_common.h"    // for LOW / HIGH
#include "main.h"


#define NM_BUS_MAX_TRX_SZ	256

/* Declare STM32 SPIx communication handler variable to winc1500 */
//SPI_HandleTypeDef hspi;
extern SPI_HandleTypeDef SPI_WIFI_HANDLE;
/* spi_rw variables */
static uint8 spiDummyBuf[300] = {0};

static void spi_select_slave(const uint8_t select);  // forward declaration
void nm_spi_cs_assert(void)   { spi_select_slave(true);  }
void nm_spi_cs_deassert(void) { spi_select_slave(false); }

tstrNmBusCapabilities egstrNmBusCapabilities =
{
	NM_BUS_MAX_TRX_SZ
};

#ifdef CONF_WINC_USE_SPI
/*
*	@fn		spi_select_slave
*	@brief	Select slave chip select: true - select, false - deselect
*	@return	None
*/
static void spi_select_slave(const uint8_t select)
{
    if (select)
    {
        HAL_GPIO_WritePin(SPI_WIFI_CS_GPIO_PORT,SPI_WIFI_CS_PIN,GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(SPI_WIFI_CS_GPIO_PORT,SPI_WIFI_CS_PIN,GPIO_PIN_SET);
    }
}

/*
*	@fn		spi_rw
*	@brief	transmit and/or receive data buffer via spi
*	@return	status
*/

#if 0

//struct spi_module master;
//struct spi_slave_inst slave_inst;

static sint8 spi_rw(uint8* pu8Mosi, uint8* pu8Miso, uint16 u16Sz)
{
	uint8 u8Dummy = 0;
	uint8 u8SkipMosi = 0, u8SkipMiso = 0;
	uint8_t txd_data = 0;
	uint8_t rxd_data = 0;

	if (!pu8Mosi) {
		pu8Mosi = &u8Dummy;
		u8SkipMosi = 1;
	}
	else if(!pu8Miso) {
		pu8Miso = &u8Dummy;
		u8SkipMiso = 1;
	}
	else {
		return M2M_ERR_BUS_FAIL;
	}

	spi_select_slave(true);


	while (u16Sz) {
		txd_data = *pu8Mosi;
		//printf("\nsend %d",txd_data);
		HAL_SPI_TransmitReceive(&SPI_WIFI_HANDLE ,&txd_data,&rxd_data,1,1000);
		//HAL_SPI_Transmit(&hspi1,&txd_data,1,1000);
		//HAL_SPI_Receive(&hspi1,&rxd_data,1,1000);
//		while (!spi_is_ready_to_write(&master))
//			;
//		while(spi_write(&master, txd_data) != STATUS_OK)
//			;

//		/* Read SPI master data register. */
//		while (!spi_is_ready_to_read(&master))
//			;
//		while (spi_read(&master, &rxd_data) != STATUS_OK)
//			;
		*pu8Miso = rxd_data;
//printf("\nrecv %d",rxd_data);
		u16Sz--;
		if (!u8SkipMiso)
			pu8Miso++;
		if (!u8SkipMosi)
			pu8Mosi++;
	}



//	while (!spi_is_write_complete(&master))
//		;

spi_select_slave(false);

	return M2M_SUCCESS;
}
#else

static sint8 spi_rw(uint8* pu8Mosi, uint8* pu8Miso, uint16 u16Sz)
{
   HAL_StatusTypeDef status;
   
    /* Start SPI transaction - polling method */
  //	spi_select_slave(true);
    
    
    /* Transmit/Recieve */
    if (pu8Mosi == NULL)
	{
		status = HAL_SPI_TransmitReceive(&SPI_WIFI_HANDLE,spiDummyBuf,pu8Miso,u16Sz,1000);
    }
    else if(pu8Miso == NULL)
    {
        status = HAL_SPI_TransmitReceive(&SPI_WIFI_HANDLE,pu8Mosi,spiDummyBuf,u16Sz,1000);
        memset(spiDummyBuf,0, u16Sz);
    }
    else
    {     
        status = HAL_SPI_TransmitReceive(&SPI_WIFI_HANDLE,pu8Mosi,pu8Miso,u16Sz,1000);
    } 
    


    /* Handle Transmit/Recieve error */
    if (status != HAL_OK)
    {
        M2M_ERR("%s: HAL_SPI_TransmitReceive failed. error (%d)\n",__FUNCTION__,status);
        return status;
    }
    
  //	spi_select_slave(false);

	return M2M_SUCCESS;
}
#endif
#endif //CONF_WINC_USE_SPI

/**
 * @brief  Expose nm_spi_rw to the WINC1500 driver.
 */
sint8 nm_spi_rw(uint8* pu8Mosi, uint8* pu8Miso, uint16 u16Sz)
{
    return spi_rw(pu8Mosi, pu8Miso, u16Sz);
}





void nm_bus_wifi_spi_init(SPI_HandleTypeDef *SPI_WIFI_HANDLE )
{
    GPIO_InitTypeDef  GPIO_InitStruct;


    /* Peripheral clock enable */
    SPI_WIFI_CLK_ENABLE();

    /* ------------------------------------------------------------------
     * Chip-select  (PC5 →  WINC1500 nCS)
     * ------------------------------------------------------------------ */
    GPIO_InitStruct.Pin   = SPI_WIFI_CS_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(SPI_WIFI_CS_GPIO_PORT, &GPIO_InitStruct);   /* now GPIOC */
    HAL_GPIO_WritePin(SPI_WIFI_CS_GPIO_PORT, SPI_WIFI_CS_PIN, GPIO_PIN_SET);

    /* ------------------------------------------------------------------
     * SPI1 on PA5/PA6/PA7  (AF5)  →  WINC1500 SCK/MISO/MOSI
     * ------------------------------------------------------------------ */
//    GPIO_InitStruct.Pin       = SPI_WIFI_SCK_PIN | SPI_WIFI_MISO_PIN | SPI_WIFI_MOSI_PIN;
//    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull      = GPIO_NOPULL;
//    GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
//    GPIO_InitStruct.Alternate = SPI_WIFI_AF;                  /* AF5-SPI1 */
//    HAL_GPIO_Init(SPI_WIFI_SCK_GPIO_PORT, &GPIO_InitStruct);


    // SCK + MOSI
    GPIO_InitStruct.Pin       = SPI_WIFI_SCK_PIN | SPI_WIFI_MOSI_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = SPI_WIFI_AF;
    HAL_GPIO_Init(SPI_WIFI_SCK_GPIO_PORT, &GPIO_InitStruct);

    // MISO (needs pull-up)
    GPIO_InitStruct.Pin       = SPI_WIFI_MISO_PIN;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    HAL_GPIO_Init(SPI_WIFI_MISO_GPIO_PORT, &GPIO_InitStruct);

}
sint8 nm_bus_init(void *pvinit)
{
    HAL_SPI_DeInit(&SPI_WIFI_HANDLE);

    // Configure SPI pins & CS first
    nm_bus_wifi_spi_init(NULL);

    // Init SPI peripheral at LOW speed for safe wake
    SPI_WIFI_HANDLE.Instance               = SPI_WIFI;
    SPI_WIFI_HANDLE.Init.Mode              = SPI_MODE_MASTER;
    SPI_WIFI_HANDLE.Init.Direction         = SPI_DIRECTION_2LINES;
    SPI_WIFI_HANDLE.Init.DataSize          = SPI_DATASIZE_8BIT;
    SPI_WIFI_HANDLE.Init.CLKPolarity       = SPI_POLARITY_LOW;
    SPI_WIFI_HANDLE.Init.CLKPhase          = SPI_PHASE_1EDGE;
    SPI_WIFI_HANDLE.Init.NSS               = SPI_NSS_SOFT;
    SPI_WIFI_HANDLE.Init.BaudRatePrescaler = CONF_WINC_SPI_LOW_BAUD_PRESCALER;
    SPI_WIFI_HANDLE.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    SPI_WIFI_HANDLE.Init.TIMode            = SPI_TIMODE_DISABLE;
    SPI_WIFI_HANDLE.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    SPI_WIFI_HANDLE.Init.CRCPolynomial     = 0x7;
    SPI_WIFI_HANDLE.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
    SPI_WIFI_HANDLE.Init.FifoThreshold     = SPI_FIFO_THRESHOLD_01DATA;
    SPI_WIFI_HANDLE.Init.MasterSSIdleness           = SPI_MASTER_SS_IDLENESS_00CYCLE;
    SPI_WIFI_HANDLE.Init.MasterInterDataIdleness    = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
    SPI_WIFI_HANDLE.Init.MasterReceiverAutoSusp     = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    SPI_WIFI_HANDLE.Init.MasterKeepIOState           = SPI_MASTER_KEEP_IO_STATE_DISABLE;
    SPI_WIFI_HANDLE.Init.IOSwap                      = SPI_IO_SWAP_DISABLE;
    SPI_WIFI_HANDLE.Init.ReadyMasterManagement       = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
    SPI_WIFI_HANDLE.Init.ReadyPolarity               = SPI_RDY_POLARITY_HIGH;

    if (HAL_SPI_Init(&SPI_WIFI_HANDLE) != HAL_OK) {
        M2M_ERR("SPI bus init error\r\n");
        return M2M_ERR_BUS_FAIL;
    }

    M2M_INFO("SPI bus init OK, prescaler=%d\r\n",
             (int)SPI_WIFI_HANDLE.Init.BaudRatePrescaler);

    // In nm_bus_init(), immediately after HAL_SPI_Init():
    if (HAL_SPI_Init(&SPI_WIFI_HANDLE) != HAL_OK) {
        M2M_ERR("SPI bus init error\r\n");
        return M2M_ERR_BUS_FAIL;
    }


    // *** DEBUG: print raw register values and verify FIFO is empty ***
    M2M_ERR("[DBG nm_bus_init] SPI1 CR1=0x%08lX CFG1=0x%08lX CFG2=0x%08lX SR=0x%08lX\r\n",
        (unsigned long)SPI1->CR1,
        (unsigned long)SPI1->CFG1,
        (unsigned long)SPI1->CFG2,
        (unsigned long)SPI1->SR);
    M2M_ERR("[DBG nm_bus_init] CS pin state = %d\r\n",
        (int)HAL_GPIO_ReadPin(SPI_WIFI_CS_GPIO_PORT, SPI_WIFI_CS_PIN));
    M2M_ERR("[DBG nm_bus_init] CHIP_EN pin state = %d\r\n",
        (int)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1));
    M2M_ERR("[DBG nm_bus_init] RESET pin state = %d\r\n",
        (int)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0));

    // Drain any stale bytes from the FIFO
    while (__HAL_SPI_GET_FLAG(&SPI_WIFI_HANDLE, SPI_FLAG_RXWNE)) {
        volatile uint32_t dummy = SPI_WIFI_HANDLE.Instance->RXDR;
        (void)dummy;
    }
    return M2M_SUCCESS;
}

/*
*	@fn		nm_bus_init
*	@brief	Initialize the bus wrapper
*	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*/

//sint8 nm_bus_init(void *pvinit)
//{
//	sint8 result = M2M_SUCCESS;
//
//	 /* WiFi SPI init function - called from nm_bus_init() */
//
//	SPI_WIFI_HANDLE.Instance			   = SPI_WIFI;
//	SPI_WIFI_HANDLE.Init.Mode			   = SPI_MODE_MASTER;
//	SPI_WIFI_HANDLE.Init.Direction 	   = SPI_DIRECTION_2LINES;
//	SPI_WIFI_HANDLE.Init.DataSize		   = SPI_DATASIZE_8BIT;
//	SPI_WIFI_HANDLE.Init.CLKPolarity	   = SPI_POLARITY_LOW;
//	SPI_WIFI_HANDLE.Init.CLKPhase		   = SPI_PHASE_1EDGE;
//	SPI_WIFI_HANDLE.Init.NSS			   = SPI_NSS_SOFT;
//	SPI_WIFI_HANDLE.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
//	SPI_WIFI_HANDLE.Init.FirstBit		   = SPI_FIRSTBIT_MSB;
//	SPI_WIFI_HANDLE.Init.TIMode		   = SPI_TIMODE_DISABLE;
//	SPI_WIFI_HANDLE.Init.CRCCalculation   = SPI_CRCCALCULATION_DISABLE;
//	SPI_WIFI_HANDLE.Init.CRCPolynomial    = 10;
//	SPI_WIFI_HANDLE.Init.CRCLength		 = SPI_CRC_LENGTH_DATASIZE;
//	SPI_WIFI_HANDLE.Init.NSSPMode		 = SPI_NSS_PULSE_DISABLE;
//	if (HAL_SPI_Init(&SPI_WIFI_HANDLE) != HAL_OK)
//	{
//		M2M_ERR("SPI bus Initialization error\r\n");
//	}
//
//	HAL_SPI_MspInit(&SPI_WIFI_HANDLE);
//	return result;
//}

/*
*	@fn		nm_bus_ioctl
*	@brief	send/receive from the bus
*	@param[IN]	u8Cmd
*					IOCTL command for the operation
*	@param[IN]	pvParameter
*					Arbitrary parameter depenging on IOCTL
*	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*	@note	For SPI only, it's important to be able to send/receive at the same time
*/
sint8 nm_bus_ioctl(uint8 u8Cmd, void* pvParameter)
{
	sint8 s8Ret = 0;
	switch(u8Cmd)
	{
		case NM_BUS_IOCTL_RW: {
			tstrNmSpiRw *pstrParam = (tstrNmSpiRw *)pvParameter;
			s8Ret = spi_rw(pstrParam->pu8InBuf, pstrParam->pu8OutBuf, pstrParam->u16Sz);

		}
		break;
		default:
			s8Ret = -1;
			M2M_ERR("invalide ioclt cmd\n");
			break;
	}

	return s8Ret;
}

/*
*	@fn		nm_bus_deinit
*	@brief	De-initialize the bus wrapper
*/
sint8 nm_bus_deinit(void)
{
	return M2M_SUCCESS;
}

/*
*	@fn			nm_bus_reinit
*	@brief		re-initialize the bus wrapper
*	@param [in]	void *config
*					re-init configuration data
*	@return		M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*/
sint8 nm_bus_reinit(void* config)
{
	return M2M_SUCCESS;
}

sint8 nm_bus_speed(uint8 u8Speed)
{
    /* de-init current SPI bus */
    HAL_SPI_DeInit(&SPI_WIFI_HANDLE);

    /* pick either slow or fast prescaler */
    if (u8Speed == LOW) {
        SPI_WIFI_HANDLE.Init.BaudRatePrescaler = CONF_WINC_SPI_LOW_BAUD_PRESCALER;
    } else {
        SPI_WIFI_HANDLE.Init.BaudRatePrescaler = CONF_WINC_SPI_BAUD_PRESCALER;
    }

    /* re-init SPI with new speed */
    HAL_SPI_Init(&SPI_WIFI_HANDLE);

    return M2M_SUCCESS;
}
