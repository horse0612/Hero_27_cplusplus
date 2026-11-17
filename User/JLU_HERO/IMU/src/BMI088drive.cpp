#include "BMI088driver.hpp"
#include "BMI088Middleware.hpp"
#include "BMI088reg.hpp"
#include "bsp_dwt.hpp"
#include "libxr.hpp"
#include "main.h"
#include "xr_hardware.hpp"
#include <cmath>
#include <cstring>
extern "C"
{

float BMI088_ACCEL_SEN = 24.0f * 9.80476f / 32768.0f;
float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;
IMU_Data_t BMI088 = {};
}

namespace
{

LibXR::Topic& BMI088SampleTopic()
{
    static LibXR::Topic topic = LibXR::Topic::CreateTopic<IMU_Data_t>("/imu/bmi088/sample");
    return topic;
}

uint8_t read_value = 0;
uint8_t write_index = 0;
uint8_t driver_error = BMI088_NO_ERROR;
enum class AsyncTransfer : uint8_t
{
    None,
    Gyro,
    Accel,
};

AsyncTransfer async_transfer = AsyncTransfer::None;
uint8_t async_rx[19] = {};
uint8_t async_command = 0U;
uint8_t async_temperature_count = 0U;
bool async_temperature_read = false;
bool async_gyro_ready = false;
LibXR::MicrosecondTimestamp async_timestamp = {};
float async_gyro[3] = {};

void AsyncTransferComplete(bool in_isr, void*, LibXR::ErrorCode result);

LibXR::WriteOperation::Callback& AsyncCallback()
{
    static auto callback = LibXR::WriteOperation::Callback::Create(AsyncTransferComplete, static_cast<void*>(nullptr));
    return callback;
}


void AsyncDeselect(AsyncTransfer transfer)
{
    
    switch (transfer)
    {
       
        case (AsyncTransfer::Gyro):
        {
            
            if (auto* cs = XRobotHardware::GyroCs(); cs != nullptr)
            cs->Write(true);
            break;
        }
      
        case (AsyncTransfer::Accel ):
        {
        
            if (auto* cs = XRobotHardware::AccelCs(); cs != nullptr)
            cs->Write(true);
            break;
        }
    default:
        break;
    }
}

bool AsyncStartGyro(bool in_isr)
{
    auto* spi = XRobotHardware::BMI088Spi();
    auto* cs = XRobotHardware::GyroCs();
    if (spi == nullptr || cs == nullptr || async_transfer != AsyncTransfer::None)
    {
        return false;
    }

    async_transfer = AsyncTransfer::Gyro;
    async_gyro_ready = false;
    async_timestamp = LibXR::Timebase::GetMicroseconds();
    async_command = static_cast<uint8_t>(BMI088_GYRO_CHIP_ID | 0x80) | 0x02;
    cs->Write(false);
    LibXR::WriteOperation operation(AsyncCallback());
    const auto result = spi->ReadAndWrite({async_rx, 7}, {&async_command, 1}, operation, in_isr);
    if (result == LibXR::ErrorCode::OK)
    {
        return true;
    }

    AsyncDeselect(async_transfer);
    async_transfer = AsyncTransfer::None;
    return false;
}

void AsyncPublish(bool in_isr)
{
    if (!async_gyro_ready)
    {
        return;
    }

    IMU_Data_t sample = BMI088;
    sample.Gyro[0] = async_gyro[0];
    sample.Gyro[1] = async_gyro[1];
    sample.Gyro[2] = async_gyro[2];

    const int16_t x = static_cast<int16_t>(static_cast<uint16_t>(async_rx[2]) |
                                           (static_cast<uint16_t>(async_rx[3]) << 8U));
    const int16_t y = static_cast<int16_t>(static_cast<uint16_t>(async_rx[4]) |
                                           (static_cast<uint16_t>(async_rx[5]) << 8U));
    const int16_t z = static_cast<int16_t>(static_cast<uint16_t>(async_rx[6]) |
                                           (static_cast<uint16_t>(async_rx[7]) << 8U));
    sample.Accel[0] = static_cast<float>(x) * BMI088_ACCEL_SEN * BMI088.AccelScale;
    sample.Accel[1] = static_cast<float>(y) * BMI088_ACCEL_SEN * BMI088.AccelScale;
    sample.Accel[2] = static_cast<float>(z) * BMI088_ACCEL_SEN * BMI088.AccelScale;

    if (async_temperature_read)
    {
        int16_t raw = static_cast<int16_t>((static_cast<uint16_t>(async_rx[17]) << 3U) |
                                            (static_cast<uint16_t>(async_rx[18]) >> 5U));
        if (raw > 1023) raw = static_cast<int16_t>(raw - 2048);
        sample.Temperature = static_cast<float>(raw) * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
    }

    BMI088 = sample;
    BMI088SampleTopic().PublishFromCallback(BMI088, async_timestamp, in_isr);
    async_gyro_ready = false;
}

void AsyncTransferComplete(bool in_isr, void*, LibXR::ErrorCode result)
{

    const AsyncTransfer completed = async_transfer;
    AsyncDeselect(completed);

    if (result != LibXR::ErrorCode::OK)
    {
        async_transfer = AsyncTransfer::None;
        async_gyro_ready = false;
        async_temperature_read = false;
        return;
    }

    if (completed == AsyncTransfer::Gyro)
    {
        const int16_t x = static_cast<int16_t>(static_cast<uint16_t>(async_rx[1]) |
                                               (static_cast<uint16_t>(async_rx[2]) << 8U));
        const int16_t y = static_cast<int16_t>(static_cast<uint16_t>(async_rx[3]) |
                                               (static_cast<uint16_t>(async_rx[4]) << 8U));
        const int16_t z = static_cast<int16_t>(static_cast<uint16_t>(async_rx[5]) |
                                               (static_cast<uint16_t>(async_rx[6]) << 8U));
        async_gyro[0] = static_cast<float>(x) * BMI088_GYRO_SEN - BMI088.GyroOffset[0];
        async_gyro[1] = static_cast<float>(y) * BMI088_GYRO_SEN - BMI088.GyroOffset[1];
        async_gyro[2] = static_cast<float>(z) * BMI088_GYRO_SEN - BMI088.GyroOffset[2];
        async_gyro_ready = true;

        auto* spi = XRobotHardware::BMI088Spi();
        auto* cs = XRobotHardware::AccelCs();
        if (spi == nullptr || cs == nullptr)
        {
            async_transfer = AsyncTransfer::None;
            async_gyro_ready = false;
            return;
        }

        const uint8_t next_count = static_cast<uint8_t>(async_temperature_count + 1U);
        async_temperature_read = next_count >= 100U;
        async_temperature_count = async_temperature_read ? 0U : next_count;
        async_transfer = AsyncTransfer::Accel;
        async_command = static_cast<uint8_t>(BMI088_ACCEL_XOUT_L | 0x80U);
        cs->Write(false);
        LibXR::WriteOperation operation(AsyncCallback());
        const size_t size = async_temperature_read ? 19U : 8U;
        if (spi->ReadAndWrite({async_rx, size}, {&async_command, 1U}, operation, in_isr) == LibXR::ErrorCode::OK)
        {
            return;
        }

        AsyncDeselect(async_transfer);
        async_transfer = AsyncTransfer::None;
        async_gyro_ready = false;
        async_temperature_read = false;
        return;
    }

    if (completed == AsyncTransfer::Accel)
    {
     
        AsyncPublish(in_isr);
        async_transfer = AsyncTransfer::None;
        async_temperature_read = false;
    }
}

void AsyncDataReady(bool in_isr)
{
    if (async_transfer == AsyncTransfer::None)
    {
        static_cast<void>(AsyncStartGyro(in_isr));
    }
}

void AsyncDataReadyCallback(bool in_isr, void*)
{
    AsyncDataReady(in_isr);
}

const uint8_t accel_register_config[BMI088_WRITE_ACCEL_REG_NUM][3] = {{BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_1600_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR}, {BMI088_ACC_RANGE, BMI088_ACC_RANGE_24G, BMI088_ACC_RANGE_ERROR}, {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR}, {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}};

const uint8_t gyro_register_config[BMI088_WRITE_GYRO_REG_NUM][3] = {{BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR}, {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_2000_230_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR}, {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR}, {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT4_GPIO_OD | BMI088_GYRO_INT4_GPIO_HIGH, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR}, {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}};

bool WriteRegister(bool accel, uint8_t reg, uint8_t data)
{
    const uint8_t tx[2] = {reg, data};
    return BMI088_SPI_Transfer(accel ? 1U : 0U, tx, nullptr, sizeof(tx)) != 0U;
}

bool ReadAccelRegisters(uint8_t reg, uint8_t *buffer, uint8_t length)
{
    constexpr uint8_t kMaxDataLength = 30U;
    if (buffer == nullptr || length == 0U || length > kMaxDataLength)
    {
        return false;
    }

    uint8_t tx[32] = {};
    uint8_t rx[32] = {};
    const uint8_t transaction_length = static_cast<uint8_t>(length + 2U);
    tx[0] = static_cast<uint8_t>(reg | 0x80U);
    for (uint8_t i = 1U; i < transaction_length; ++i)
    {
        tx[i] = 0x55U;
    }

    if (BMI088_SPI_Transfer(1U, tx, rx, transaction_length) == 0U)
    {
        return false;
    }
    std::memcpy(buffer, rx + 2U, length);
    return true;
}

bool ReadGyroRegisters(uint8_t reg, uint8_t *buffer, uint8_t length)
{
    constexpr uint8_t kMaxDataLength = 31U;
    if (buffer == nullptr || length == 0U || length > kMaxDataLength)
    {
        return false;
    }

    uint8_t tx[32] = {};
    uint8_t rx[32] = {};
    const uint8_t transaction_length = static_cast<uint8_t>(length + 1U);
    tx[0] = static_cast<uint8_t>(reg | 0x80U);
    for (uint8_t i = 1U; i < transaction_length; ++i)
    {
        tx[i] = 0x55U;
    }

    if (BMI088_SPI_Transfer(0U, tx, rx, transaction_length) == 0U)
    {
        return false;
    }
    std::memcpy(buffer, rx + 1U, length);
    return true;
}

bool AccelWriteRegister(uint8_t reg, uint8_t data)
{
    return WriteRegister(true, reg, data);
}

bool AccelReadRegister(uint8_t reg, uint8_t &data)
{
    return ReadAccelRegisters(reg, &data, 1U);
}

bool GyroWriteRegister(uint8_t reg, uint8_t data)
{
    return WriteRegister(false, reg, data);
}

bool GyroReadRegister(uint8_t reg, uint8_t &data)
{
    return ReadGyroRegisters(reg, &data, 1U);
}

} // namespace

LibXR::Topic& BMI088_GetSampleTopic()
{
    return BMI088SampleTopic();
}

extern "C"
{
uint8_t bmi088_accel_init(void)
{
    if (!AccelReadRegister(BMI088_ACC_CHIP_ID, read_value))
    return BMI088_NO_SENSOR;

    if (!AccelReadRegister(BMI088_ACC_CHIP_ID, read_value) || read_value != BMI088_ACC_CHIP_ID_VALUE)
    return BMI088_NO_SENSOR;

    if (!AccelWriteRegister(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE))
    return BMI088_NO_SENSOR;

    LibXR::Timebase::DelayMicroseconds(1000);

    static_cast<void>(AccelReadRegister(BMI088_ACC_CHIP_ID, read_value));
    if (!AccelReadRegister(BMI088_ACC_CHIP_ID, read_value) || read_value != BMI088_ACC_CHIP_ID_VALUE) return BMI088_NO_SENSOR;

    for (uint8_t attempt = 0; attempt < 3U; ++attempt)
    {
        if (!AccelWriteRegister(BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE)) return BMI088_NO_SENSOR;
        LibXR::Timebase::DelayMicroseconds(1000);
        if (AccelReadRegister(BMI088_ACC_PWR_CONF, read_value) && read_value == BMI088_ACC_PWR_ACTIVE_MODE) break;
        if (attempt == 2U) return BMI088_NO_SENSOR;
    }
    if (!AccelWriteRegister(BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON)) return BMI088_NO_SENSOR;
    LibXR::Timebase::DelayMicroseconds(450);
    if (!AccelReadRegister(BMI088_ACC_PWR_CTRL, read_value) || read_value != BMI088_ACC_ENABLE_ACC_ON) return BMI088_NO_SENSOR;

    for (write_index = 0; write_index < BMI088_WRITE_ACCEL_REG_NUM; ++write_index)
    {
        bool ok = false;
        for (uint8_t attempt = 0; attempt < 3U; ++attempt)
        {
            ok = AccelWriteRegister(accel_register_config[write_index][0], accel_register_config[write_index][1]);
            LibXR::Timebase::DelayMicroseconds(2);
            ok = ok && AccelReadRegister(accel_register_config[write_index][0], read_value) &&
                 read_value == accel_register_config[write_index][1];
            if (ok) break;
        }
        if (!ok) driver_error |= accel_register_config[write_index][2];
    }
    return driver_error == BMI088_NO_ERROR ? BMI088_NO_ERROR : driver_error;
}

uint8_t bmi088_gyro_init(void)
{
    if (!GyroReadRegister(BMI088_GYRO_CHIP_ID, read_value) || read_value != BMI088_GYRO_CHIP_ID_VALUE)
    return BMI088_NO_SENSOR;

    if (!GyroWriteRegister(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE))
    return BMI088_NO_SENSOR;
    LibXR::Timebase::DelayMicroseconds(30000);
    if (!GyroReadRegister(BMI088_GYRO_CHIP_ID, read_value) || read_value != BMI088_GYRO_CHIP_ID_VALUE) return BMI088_NO_SENSOR;

    for (write_index = 0; write_index < BMI088_WRITE_GYRO_REG_NUM; ++write_index)
    {
        bool ok = false;
        for (uint8_t attempt = 0; attempt < 3U; ++attempt)
        {
            ok = GyroWriteRegister(gyro_register_config[write_index][0], gyro_register_config[write_index][1]);
            LibXR::Timebase::DelayMicroseconds(2);
            ok = ok && GyroReadRegister(gyro_register_config[write_index][0], read_value) &&
                 read_value == gyro_register_config[write_index][1];
            if (ok) break;
        }
        if (!ok) driver_error |= gyro_register_config[write_index][2];
    }
    return driver_error == BMI088_NO_ERROR ? BMI088_NO_ERROR : driver_error;
}

uint8_t BMI088_init(uint8_t calibrate)
{
    LibXR::Timebase::DelayMicroseconds(30000);
    driver_error = BMI088_NO_ERROR;
    uint8_t accel_error = bmi088_accel_init();
    uint8_t gyro_error = bmi088_gyro_init();
    driver_error |= accel_error;
    driver_error |= gyro_error;
    if (calibrate != 0)
    {
        static_cast<void>(BMI088_CalibrateOffset(10000));
    }
    else
    {
    BMI088.GyroOffset[0] = GxOFFSET;
    BMI088.GyroOffset[1] = GyOFFSET;
    BMI088.GyroOffset[2] = GzOFFSET;
    BMI088.gNorm = gNORM;
    BMI088.AccelScale = 9.81f / BMI088.gNorm;
    }
    return driver_error;
}

uint8_t BMI088_CalibrateOffset(uint16_t sample_count)
{

    float sum[3] = {};
    float min_value[3] = {};
    float max_value[3] = {};
    float norm_sum = 0.0F;
    float norm_min = 0.0F;
    float norm_max = 0.0F;
    bool first = true;
    uint8_t accel_buffer[6] = {};
    uint8_t gyro_buffer[6] = {};

    for (uint16_t i = 0U; i < sample_count; ++i)
    {
        if (!ReadAccelRegisters(BMI088_ACCEL_XOUT_L, accel_buffer, sizeof(accel_buffer)) ||!ReadGyroRegisters(BMI088_GYRO_X_L, gyro_buffer, sizeof(gyro_buffer)))
        {
            return 0;
        }

        float accel_squared_sum = 0.0F;
        for (uint8_t axis = 0U; axis < 3U; ++axis)
        {
            const uint8_t index = static_cast<uint8_t>(axis * 2U);
            const int16_t raw = static_cast<int16_t>(
                static_cast<uint16_t>(accel_buffer[index]) |
                (static_cast<uint16_t>(accel_buffer[index + 1U]) << 8U));
            const float value = static_cast<float>(raw) * BMI088_ACCEL_SEN;
            accel_squared_sum += value * value;
        }
        const float accel_norm = std::sqrt(accel_squared_sum);
        norm_sum += accel_norm;
        if (first)
        {
            norm_min = accel_norm;
            norm_max = accel_norm;
        }
        else
        {
            if (accel_norm < norm_min) norm_min = accel_norm;
            if (accel_norm > norm_max) norm_max = accel_norm;
        }

        for (uint8_t axis = 0U; axis < 3U; ++axis)
        {
            const uint8_t index = static_cast<uint8_t>(axis * 2U);
            const int16_t raw = static_cast<int16_t>(
                static_cast<uint16_t>(gyro_buffer[index]) |
                (static_cast<uint16_t>(gyro_buffer[index + 1U]) << 8U));
            const float value = static_cast<float>(raw) * BMI088_GYRO_SEN;
            sum[axis] += value;
            if (first)
            {
                min_value[axis] = value;
                max_value[axis] = value;
            }
            else
            {
                if (value < min_value[axis]) min_value[axis] = value;
                if (value > max_value[axis]) max_value[axis] = value;
            }
        }
        first = false;
        HAL_Delay(1);
    }

    float spread = 0.0F;
    for (uint8_t axis = 0U; axis < 3U; ++axis)
    {
        const float axis_spread = max_value[axis] - min_value[axis];
        if (axis_spread > spread) spread = axis_spread;
    }
    const float norm_spread = norm_max - norm_min;
    const float inverse_count = 1.0F / static_cast<float>(sample_count);
    for (uint8_t axis = 0U; axis < 3U; ++axis)
    {
        BMI088.GyroOffset[axis] = sum[axis] * inverse_count;
    }
    BMI088.gNorm = norm_sum * inverse_count;

    if (spread > 0.15F || norm_spread > 0.5F)
    {
        return 0U;
    }

    if (BMI088.gNorm <= 8.0F || BMI088.gNorm >= 12.0F)
    {
        BMI088.AccelScale = 1.0F;
        return 0U;
    }
    BMI088.AccelScale = 9.81F / BMI088.gNorm;
    return 1U;
}

uint8_t BMI088_StartAsync(void)
{
    auto* gyro_int = XRobotHardware::GyroInt();
    if (gyro_int == nullptr)
    {
        return BMI088_NO_SENSOR;
    }

    static_cast<void>(AsyncCallback());

    if (gyro_int->RegisterCallback(LibXR::GPIO::Callback::Create(AsyncDataReadyCallback, static_cast<void*>(nullptr))) != LibXR::ErrorCode::OK)
    {
        return BMI088_NO_SENSOR;
    }

    if (gyro_int->EnableInterrupt() != LibXR::ErrorCode::OK)
    {
        return BMI088_NO_SENSOR;
    }

    return BMI088_NO_ERROR;
}

}
