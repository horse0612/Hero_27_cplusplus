#include "LQR.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
//本文件用于电机的LQR控制器设计，主要是对状态空间模型进行求解，得到最优控制律
namespace
{
bool Finite(const float *data, std::size_t count)
{
    if (!data) return false;
    for (std::size_t i = 0; i < count; ++i)
        if (!std::isfinite(data[i])) return false;
    return true;
}

bool Dimensions(std::size_t n, std::size_t m)
{
    return n > 0 && n <= LQR::MaxStates && m > 0 && m <= LQR::MaxInputs;
}

// Cholesky 分解；允许 Q 存在零主元，R 和控制 Hessian 必须严格正定。
bool Cholesky(double *matrix, std::size_t n, bool semidefinite)
{
    double scale = 0;
    for (std::size_t i = 0; i < n * n; ++i)
        scale = std::max(scale, std::abs(matrix[i]));
    const double eps = scale * 1.0e-12;
    for (std::size_t i = 0; i < n; ++i)
    {
        for (std::size_t j = 0; j <= i; ++j)
        {
            double value = matrix[i * n + j];
            for (std::size_t k = 0; k < j; ++k)
                value -= matrix[i * n + k] * matrix[j * n + k];
            if (!std::isfinite(value)) return false;
            if (i == j)
            {
                if (semidefinite ? value < -eps : value <= eps) return false;
                matrix[i * n + j] = value <= eps ? 0 : std::sqrt(value);
            }
            else if (matrix[j * n + j] > 0)
                matrix[i * n + j] = value / matrix[j * n + j];
            else
            {
                if (std::abs(value) > eps) return false;
                matrix[i * n + j] = 0;
            }
        }
    }
    return true;
}

bool Weight(const float *matrix, std::size_t n, bool semidefinite)
{
    double copy[LQR::MaxStates * LQR::MaxStates] = {};
    double scale = 0;
    for (std::size_t i = 0; i < n * n; ++i)
        scale = std::max(scale, std::abs(static_cast<double>(matrix[i])));
    for (std::size_t i = 0; i < n; ++i)
        for (std::size_t j = 0; j < n; ++j)
        {
            if (std::abs(static_cast<double>(matrix[i*n+j]) - matrix[j*n+i]) > scale * 1.e-6)
                return false;
            copy[i*n+j] = (static_cast<double>(matrix[i*n+j]) + matrix[j*n+i]) * 0.5;
        }
    return Cholesky(copy, n, semidefinite);
}
} // namespace

LQR::Status LQR::Init(std::size_t states, std::size_t inputs,
                      const float *a, const float *b, const float *q, const float *r)
{
    ready_ = model_ = limited_ = false;
    n_ = m_ = iterations_ = 0;
    if (!Dimensions(states, inputs) || !Finite(a, states*states) ||
        !Finite(b, states*inputs) || !Finite(q, states*states) || !Finite(r, inputs*inputs))
        return Status::InvalidArgument;
    if (!Weight(q, states, true) || !Weight(r, inputs, false)) return Status::InvalidWeights;
    n_ = states;
    m_ = inputs;
    std::copy_n(a, n_*n_, a_);
    std::copy_n(b, n_*m_, b_);
    for (std::size_t i = 0; i < n_; ++i)
        for (std::size_t j = 0; j < n_; ++j)
            q_[i*n_+j] = q[i*n_+j]*0.5f + q[j*n_+i]*0.5f;
    for (std::size_t i = 0; i < m_; ++i)
        for (std::size_t j = 0; j < m_; ++j)
            r_[i*m_+j] = r[i*m_+j]*0.5f + r[j*m_+i]*0.5f;
    model_ = true;
    return Status::Ok;
}

LQR::Status LQR::Solve(std::size_t maxIterations, float tolerance)
{
    ready_ = false;
    iterations_ = 0;
    if (!model_) return Status::NotReady;
    if (!maxIterations || !std::isfinite(tolerance) || tolerance <= 0 || tolerance >= 1)
        return Status::InvalidArgument;
    // Double 工作区降低 Riccati 迭代抵消误差；局部工作区约 3 KiB。
    double p[MaxStates*MaxStates] = {}, next[MaxStates*MaxStates] = {};
    double pa[MaxStates*MaxStates] = {}, pb[MaxStates*MaxInputs] = {};
    double h[MaxInputs*MaxInputs] = {}, g[MaxInputs*MaxStates] = {};
    double k[MaxInputs*MaxStates] = {};
    std::copy_n(q_, n_*n_, p);
    bool converged = false;
    for (;;)
    {
        for (std::size_t i = 0; i < n_; ++i)
        {
            for (std::size_t j = 0; j < n_; ++j)
            {
                pa[i*n_+j] = 0;
                for (std::size_t t = 0; t < n_; ++t) pa[i*n_+j] += p[i*n_+t]*a_[t*n_+j];
            }
            for (std::size_t j = 0; j < m_; ++j)
            {
                pb[i*m_+j] = 0;
                for (std::size_t t = 0; t < n_; ++t) pb[i*m_+j] += p[i*n_+t]*b_[t*m_+j];
            }
        }
        for (std::size_t i = 0; i < m_; ++i)
        {
            for (std::size_t j = 0; j < m_; ++j)
            {
                h[i*m_+j] = r_[i*m_+j];
                for (std::size_t t = 0; t < n_; ++t) h[i*m_+j] += b_[t*m_+i]*pb[t*m_+j];
            }
            for (std::size_t j = 0; j < n_; ++j)
            {
                g[i*n_+j] = 0;
                for (std::size_t t = 0; t < n_; ++t) g[i*n_+j] += b_[t*m_+i]*pa[t*n_+j];
            }
        }
        // 解 (R+B'PB)K=B'PA，避免显式求逆。
        if (!Cholesky(h, m_, false)) return Status::NumericalError;
        for (std::size_t j = 0; j < n_; ++j)
        {
            for (std::size_t i = 0; i < m_; ++i)
            {
                double value = g[i*n_+j];
                for (std::size_t t = 0; t < i; ++t) value -= h[i*m_+t]*k[t*n_+j];
                k[i*n_+j] = value/h[i*m_+i];
            }
            for (std::size_t ii = m_; ii > 0; --ii)
            {
                const std::size_t i = ii-1;
                double value = k[i*n_+j];
                for (std::size_t t = i+1; t < m_; ++t) value -= h[t*m_+i]*k[t*n_+j];
                k[i*n_+j] = value/h[i*m_+i];
                if (!std::isfinite(k[i*n_+j]) || std::abs(k[i*n_+j]) > std::numeric_limits<float>::max())
                    return Status::NumericalError;
            }
        }
        if (converged)
        {
            std::copy_n(k, m_*n_, gain_);
            ready_ = true;
            return Status::Ok;
        }
        // P_next = Q + A'PA - A'PB*K。
        for (std::size_t i = 0; i < n_; ++i)
            for (std::size_t j = 0; j < n_; ++j)
            {
                double value = q_[i*n_+j];
                for (std::size_t t = 0; t < n_; ++t) value += a_[t*n_+i]*pa[t*n_+j];
                for (std::size_t t = 0; t < m_; ++t) value -= g[t*n_+i]*k[t*n_+j];
                if (!std::isfinite(value)) return Status::NumericalError;
                next[i*n_+j] = value;
            }
        double delta = 0, scale = 0;
        for (std::size_t i = 0; i < n_; ++i)
            for (std::size_t j = i; j < n_; ++j)
            {
                const double value = next[i*n_+j]*0.5 + next[j*n_+i]*0.5;
                delta = std::max(delta, std::abs(value-p[i*n_+j]));
                scale = std::max(scale, std::abs(value));
                p[i*n_+j] = p[j*n_+i] = value;
            }
        ++iterations_;
        converged = delta <= tolerance * std::max(scale, std::numeric_limits<double>::min());
        if (!converged && iterations_ >= maxIterations) return Status::NotConverged;
    }
}

LQR::Status LQR::SetGain(std::size_t states, std::size_t inputs, const float *gain)
{
    if (!Dimensions(states, inputs) || !Finite(gain, states*inputs)) return Status::InvalidArgument;
    std::copy_n(gain, states*inputs, gain_);
    n_ = states;
    m_ = inputs;
    iterations_ = 0;
    model_ = limited_ = false;
    ready_ = true;
    return Status::Ok;
}

LQR::Status LQR::SetOutputLimits(const float *lower, const float *upper)
{
    if (!n_) return Status::NotReady;
    if (!Finite(lower, m_) || !Finite(upper, m_)) return Status::InvalidArgument;
    for (std::size_t i = 0; i < m_; ++i)
        if (lower[i] > upper[i]) return Status::InvalidArgument;
    std::copy_n(lower, m_, lower_);
    std::copy_n(upper, m_, upper_);
    limited_ = true;
    return Status::Ok;
}

LQR::Status LQR::Calculate(const float *state, float *output,
                           const float *reference, const float *feedforward) const
{
    if (!ready_) return Status::NotReady;
    if (!output || !Finite(state, n_) || (reference && !Finite(reference, n_)) ||
        (feedforward && !Finite(feedforward, m_))) return Status::InvalidArgument;
    float result[MaxInputs] = {};
    for (std::size_t i = 0; i < m_; ++i)
    {
        double value = feedforward ? feedforward[i] : 0;
        for (std::size_t j = 0; j < n_; ++j)
            value -= gain_[i*n_+j]*(static_cast<double>(state[j]) - (reference ? reference[j] : 0));
        if (!std::isfinite(value)) return Status::NumericalError;
        if (limited_) value = std::max(static_cast<double>(lower_[i]), std::min(static_cast<double>(upper_[i]), value));
        if (std::abs(value) > std::numeric_limits<float>::max()) return Status::NumericalError;
        result[i] = static_cast<float>(value);
    }
    std::copy_n(result, m_, output);
    return Status::Ok;
}
