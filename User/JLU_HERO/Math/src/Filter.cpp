#include "Filter.hpp"

#include <algorithm>
#include <cstddef>

extern "C"
{

void Filter_InitAverFilter(AverFilter *filter, const std::uint16_t size)
{
    filter->bufferSize = std::min(size, AVER_FILTER_MAX_SIZE);

    for (std::uint16_t i = 0; i < filter->bufferSize; ++i)
    {
        filter->buffer[i] = 0.0f;
    }
}

float Filter_AverCalc(AverFilter *filter, const float newValue)
{
    float sum = 0.0f;

    for (std::uint16_t i = 0; i + 1U < filter->bufferSize; ++i)
    {
        filter->buffer[i] = filter->buffer[i + 1U];

        sum += filter->buffer[i];
    }

    filter->buffer[filter->bufferSize - 1U] = newValue;

    sum += newValue;

    return sum / static_cast<float>(filter->bufferSize);
}

void Filter_AverClear(AverFilter *filter)
{

    for (std::uint16_t i = 0; i < filter->bufferSize; ++i)
    {
        filter->buffer[i] = 0.0f;
    }
}

void KalmanFilter_Init(KalmanFilter *filter, const double initialState, const double initialCovariance, const double processNoise, const double measurementNoise)
{

    filter->x = initialState;
    filter->P = initialCovariance;
    filter->Q = processNoise;
    filter->R = measurementNoise;
    filter->K = 0.0;
    filter->output = 0.0f;
}

double KalmanFilter_Update(KalmanFilter *filter, const double measurement)
{
    // 预测
    const double statePrior = filter->x;
    const double covariancePrior = filter->P + filter->Q;

    // 计算卡尔曼增益
    const double denominator = covariancePrior + filter->R;

    if (denominator != 0.0)
    {
        filter->K = covariancePrior / denominator;
    }
    else
    {
        filter->K = 0.0;
    }

    // 更新状态
    filter->x = statePrior + filter->K * (measurement - statePrior);

    // 更新协方差
    filter->P = (1.0 - filter->K) * covariancePrior;

    filter->output = static_cast<float>(filter->x);

    return filter->x;
}
}