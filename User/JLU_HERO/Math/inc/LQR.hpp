#pragma once

#include <cstddef>

/** 离散无限时域 LQR，模型 x[k+1] = A*x[k] + B*u[k]。
 * 所有矩阵均为紧凑行优先 float 数组；不分配堆内存。
 * A: n*n, B: n*m, Q: n*n (对称半正定), R: m*m (对称正定)。
 * 连续模型必须按实际控制周期先离散化。求解收敛要求 (A,B) 可稳定、
 * (A,sqrt(Q)) 可检测；调用方负责模型满足这些条件。
 * Solve 仅在初始化/调参时调用，不应放入实时控制中断。
 * 同一实例不支持并发调用；建议将实例置于静态存储区。
 */
class LQR
{
public:
    static constexpr std::size_t MaxStates = 8;
    static constexpr std::size_t MaxInputs = 4;

    enum class Status
    {
        Ok,
        InvalidArgument,
        InvalidWeights,
        NotReady,
        NotConverged,
        NumericalError
    };

    // 初始化会使旧增益和限幅失效；数组复制到实例内部。
    Status Init(std::size_t states, std::size_t inputs,
                const float *a, const float *b, const float *q, const float *r);
    Status Solve(std::size_t maxIterations = 1000, float tolerance = 1.0e-6f);

    // 离线增益接口，无需 Init；K 为 inputs*states，采用 u=-K*x 符号约定。
    // 调用后清除旧模型和限幅；增益的稳定性由调用方保证。
    Status SetGain(std::size_t states, std::size_t inputs, const float *gain);
    Status SetOutputLimits(const float *lower, const float *upper);
    void ClearOutputLimits() { limited_ = false; }

    // state 必填；reference/feedforward 为空时按零处理。
    // 跟踪需满足 x_ref[k+1]=A*x_ref[k]+B*u_ref[k]，否则不保证无稳态误差。
    // 失败时不修改 output；限幅后不再保证无约束 LQR 的最优性。
    Status Calculate(const float *state, float *output,
                     const float *reference = nullptr, const float *feedforward = nullptr) const;
    bool IsReady() const { return ready_; }
    std::size_t StateSize() const { return n_; }
    std::size_t InputSize() const { return m_; }
    std::size_t Iterations() const { return iterations_; }
    // 返回紧凑行优先 K；无有效增益时返回 nullptr。
    const float *GetGain() const { return ready_ ? gain_ : nullptr; }

private:
    std::size_t n_ = 0, m_ = 0, iterations_ = 0;
    bool model_ = false, ready_ = false, limited_ = false;
    float a_[MaxStates * MaxStates] = {};
    float b_[MaxStates * MaxInputs] = {};
    float q_[MaxStates * MaxStates] = {};
    float r_[MaxInputs * MaxInputs] = {};
    float gain_[MaxInputs * MaxStates] = {};
    float lower_[MaxInputs] = {}, upper_[MaxInputs] = {};
};

/* 调用示例（一维离散积分器）：
static LQR controller;
const float A[] = {1}, B[] = {0.001f}, Q[] = {10}, R[] = {1};
// 初始化阶段：检查 Init 和 Solve 均返回 LQR::Status::Ok。
// controller.Init(1, 1, A, B, Q, R);
// controller.Solve(10000);
// 周期任务：float x[] = {measurement}, ref[] = {target}, u[1];
// if (controller.Calculate(x, u, ref) == LQR::Status::Ok) { ...使用 u[0]... }
*/
