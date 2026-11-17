#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "spi.h"

#ifdef __cplusplus
}
#endif


class Bsp_SPI
{
	public:
	explicit Bsp_SPI(SPI_HandleTypeDef* hspi_) : m_handle(hspi_) {}

	
	bool Transmit(const uint8_t* data_, const uint16_t size_) const
	{
		return HAL_SPI_Transmit(m_handle, data_, size_, 1000) == HAL_OK;
	}
	bool TransmitDMA(const uint8_t* data_, const uint16_t size_) const
	{
		return HAL_SPI_Transmit_DMA(m_handle, data_, size_) == HAL_OK;
	}

	
	bool Receive(uint8_t* data_, const uint16_t size_) const
	{
		return HAL_SPI_Receive(m_handle, data_, size_, 1000) == HAL_OK;
	}
	bool ReceiveDMA(uint8_t* data_, const uint16_t size_) const
	{
		return HAL_SPI_Receive_DMA(m_handle, data_, size_) == HAL_OK;
	}

	
	bool TransmitReceive(const uint8_t* tx_data_, uint8_t* rx_data_,
		const uint16_t tx_size_, const uint16_t rx_size_) const
	{
		return HAL_SPI_TransmitReceive(m_handle, tx_data_, rx_data_, tx_size_ + rx_size_, 1000) == HAL_OK;
	}
	bool TransmitReceiveDMA(const uint8_t* tx_data_, uint8_t* rx_data_,
		const uint16_t tx_size_, const uint16_t rx_size_) const
	{
		return HAL_SPI_TransmitReceive_DMA(m_handle, tx_data_, rx_data_, tx_size_ + rx_size_) == HAL_OK;
	}

	SPI_HandleTypeDef* GetHandle() const { return m_handle; }

	friend void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi);
	friend void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);
	friend void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);

	private:
	SPI_HandleTypeDef* m_handle;
};
