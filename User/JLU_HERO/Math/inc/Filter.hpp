#pragma once

#include <cstdint>

constexpr std::uint16_t AVER_FILTER_MAX_SIZE = 100U;

struct AverFilter
{
    float buffer[AVER_FILTER_MAX_SIZE];
    std::uint16_t bufferSize;
};

struct KalmanFilter
{
    double x;
    double P;
    double Q;
    double R;
    double K;
    float output;
};

extern "C"
{

void Filter_InitAverFilter(AverFilter *filter, std::uint16_t size);

float Filter_AverCalc(AverFilter *filter, float newValue);

void Filter_AverClear(AverFilter *filter);

void KalmanFilter_Init(KalmanFilter *filter, double initialState, double initialCovariance, double processNoise, double measurementNoise);

double KalmanFilter_Update(KalmanFilter *filter, double measurement);
}