#include "BMI088Middleware.hpp"

#include "libxr.hpp"
#include "xr_hardware.hpp"

extern "C"
{

uint8_t BMI088_SPI_Transfer(uint8_t accel, const uint8_t *tx_data, uint8_t *rx_data, uint16_t length)
{
    constexpr uint16_t kMaxTransactionLength = 32;
    if (accel > 1 || tx_data == nullptr || length == 0U || length > kMaxTransactionLength)
    {
        return 0U;
    }

    auto *spi = XRobotHardware::BMI088Spi();
    auto *cs = accel != 0U ? XRobotHardware::AccelCs() : XRobotHardware::GyroCs();
    if (spi == nullptr || cs == nullptr)
    {
        return 0U;
    }

    static LibXR::Semaphore completed;
    uint8_t receive[kMaxTransactionLength]{};
    LibXR::WriteOperation operation(completed);
    cs->Write(false);
    const auto status = spi->ReadAndWrite(LibXR::RawData(receive, length),  LibXR::ConstRawData(tx_data, length), operation, false);
    cs->Write(true);

    if (status != LibXR::ErrorCode::OK)
    {
        return 0U;
    }

    if (rx_data != nullptr)
    {
        for (uint16_t i = 0; i < length; ++i)
        {
            rx_data[i] = receive[i];
        }
    }
    return 1;
}
}
