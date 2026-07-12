#include "../include/conjugate_gradient_sle_solver.h"

#include <cmath>
#include <assert.h>

// 共轭梯度法（Conjugate Gradient, CG）线性方程组求解器。
//
// 在约束系统中，要求解的线性方程组为：
//
//     A * lambda = right
//
// 其中：
//   A      = J * W * J^T
//   J      = 约束雅可比矩阵
//   W      = 质量矩阵 M 的逆矩阵，这里以对角列向量形式存储
//   right  = 方程右侧项
//   lambda = 要求解的约束乘子
//
// 共轭梯度法适合求解对称正定矩阵的线性方程组。这里的 A 通常来自
// J * W * J^T，在约束没有严重退化、W 为正时，一般具有适合 CG 的结构。
//
// 这个实现不会显式构造完整的 A 矩阵，而是在 multiply() 中按需计算：
//
//     target = A * x = J * W * J^T * x
//
// 这样可以利用 J 的稀疏结构，避免每次迭代都存储和遍历完整的密集矩阵。
//
// 本求解器不支持带上下限的 lambda，所以传给基类 SleSolver(false)。
atg_scs::ConjugateGradientSleSolver::ConjugateGradientSleSolver()
    : atg_scs::SleSolver(false)
{
    // 最大迭代次数。CG 理论上最多 n 次可收敛到精确解（n 为未知量数量），
    // 但浮点误差、病态矩阵、退化约束都会影响实际收敛，所以这里给一个上限。
    m_maxIterations = 1000;

    // 相对误差阈值。每个残差分量 err_i 都需要小于
    // max(abs(m_maxError * right_i), m_minError)。
    m_maxError = 1E-2;

    // 绝对误差下限。right_i 接近 0 时，相对误差阈值也会接近 0，
    // 所以需要一个最小绝对误差，避免永远无法满足收敛条件。
    m_minError = 1E-3;
}

atg_scs::ConjugateGradientSleSolver::~ConjugateGradientSleSolver() {
    m_mreg0.destroy();
    m_mreg1.destroy();
    m_Ap.destroy();
    m_x.destroy();
    m_r.destroy();
    m_p.destroy();
}

bool atg_scs::ConjugateGradientSleSolver::solve(
        SparseMatrix<3> &J,
        Matrix &W,
        Matrix &right,
        Matrix *previous,
        Matrix *result)
{
    const int n = right.getHeight();

    // 所有向量都是列向量，宽度为 1，高度为未知量数量 n。
    //
    // m_x  : 当前解 lambda 的估计值
    // m_r  : 当前残差 r = right - A * x
    // m_p  : 当前搜索方向
    // m_Ap : A * p 的临时结果
    m_r.resize(1, n);
    m_p.resize(1, n);
    m_Ap.resize(1, n);

    // 默认初值 x0 = 0。
    m_x.initialize(1, n);

    // 如果调用方提供了上一帧/上一次求解的 lambda，就把它作为初值。
    // 这叫 warm start。约束系统连续模拟时，相邻帧的解通常很接近，
    // 用 previous 作为初值可以减少迭代次数。
    if (previous != nullptr && previous->getHeight() == n) {
        m_x.set(previous);
    }

    result->resize(1, n);

    // 初始化残差：
    //
    //     r0 = right - A * x0
    //
    // 先复制 right 到 m_r，再计算 A*x0 到 m_Ap，
    // 最后执行 m_r += -1 * m_Ap。
    m_r.set(&right);
    multiply(J, W, m_x, &m_Ap);
    m_r.madd(m_Ap, -1);

    // 如果初始解已经足够好，就不需要迭代。
    if (sufficientlySmall(m_r, right)) {
        goto succeeded;
    }

    // 第一轮搜索方向取残差：
    //
    //     p0 = r0
    //
    // 后续每一轮会把新残差和旧搜索方向组合成新的共轭方向。
    m_p.set(&m_r);

    for (int k = 0; k < m_maxIterations; ++k) {
        // 计算 A * p_k。
        //
        // 这是每一轮 CG 的主要成本。multiply() 内部计算：
        //
        //     A * p = J * W * J^T * p
        //
        // 而不是先显式构造 A。
        multiply(J, W, m_p, &m_Ap);

        // 当前残差长度平方：
        //
        //     r_k^T * r_k
        //
        // 用它来计算沿搜索方向 p_k 应该前进多远。
        const double rk_mag = m_r.vectorMagnitudeSquared();

        // 步长 alpha：
        //
        //     alpha_k = (r_k^T r_k) / (p_k^T A p_k)
        //
        // 含义：沿着当前搜索方向 p_k 移动 alpha_k，使二次型误差尽量下降。
        const double alpha = rk_mag / m_p.dot(m_Ap);

        // 更新当前解：
        //
        //     x_{k+1} = x_k + alpha_k * p_k
        m_x.madd(m_p, alpha);

        // 更新残差：
        //
        //     r_{k+1} = r_k - alpha_k * A * p_k
        //
        // 因为 m_Ap 已经是 A*p_k，所以这里直接复用。
        m_r.madd(m_Ap, -alpha);

        // 如果残差已经足够小，说明 A*x 已经足够接近 right。
        if (sufficientlySmall(m_r, right)) {
            goto succeeded;
        }

        // 新残差长度平方：
        //
        //     r_{k+1}^T * r_{k+1}
        const double rk1_mag = m_r.vectorMagnitudeSquared();

        // 方向混合系数 beta：
        //
        //     beta_k = (r_{k+1}^T r_{k+1}) / (r_k^T r_k)
        //
        // 这个系数让新的搜索方向和之前的搜索方向在 A 度量下保持共轭。
        const double beta = rk1_mag / rk_mag;

        // 更新搜索方向：
        //
        //     p_{k+1} = r_{k+1} + beta_k * p_k
        //
        // Matrix::pmadd(a, b) 的语义是：
        //
        //     this = a + b * this
        //
        // 所以这里等价于 m_p = m_r + beta * m_p。
        m_p.pmadd(m_r, beta);
    }

    // 达到最大迭代次数仍未满足残差阈值，认为求解失败。
    return false;

succeeded:
    // 把当前近似解 lambda 写入输出。
    result->set(&m_x);

    return true;
}

void atg_scs::ConjugateGradientSleSolver::multiply(
    SparseMatrix<3> &J,
    Matrix &W,
    Matrix &x,
    Matrix *target)
{
    // 计算：
    //
    //     target = A * x
    //            = J * W * J^T * x
    //
    // 这里故意不显式构造 A = J * W * J^T。
    // 对 CG 来说，每轮只需要 A 乘以某个向量，不一定需要完整的 A 矩阵。
    target->resize(1, x.getHeight());

    // 第一步：
    //
    //     m_mreg0 = J^T * x
    //
    // 如果 x 的维度是约束数量，那么 J^T * x 的维度是广义速度/力的维度。
    J.transposeMultiplyVector(x, &m_mreg0);

    // 第二步：
    //
    //     m_mreg1 = W .* m_mreg0
    //
    // W 是逆质量/逆转动惯量组成的对角向量，所以乘 W 等价于逐分量相乘。
    W.componentMultiply(m_mreg0, &m_mreg1);

    // 第三步：
    //
    //     target = J * m_mreg1
    //
    // 合起来就是 target = J * W * J^T * x。
    J.multiply(m_mreg1, target);
}

// 判断残差 x 相对于目标 right 是否已经足够小。
//
// 这里的 x 通常是残差：
//
//     r = right - A * lambda
//
// target 通常是 right。
//
// 对每个分量分别判断：
//
//     abs(r_i) <= max(abs(m_maxError * right_i), m_minError)
//
// 也就是说：
//   1. right_i 较大时，用相对误差 m_maxError；
//   2. right_i 接近 0 时，用绝对误差 m_minError。
bool atg_scs::ConjugateGradientSleSolver::sufficientlySmall(
    Matrix &x,
    Matrix &target) const
{
    for (int i = 0; i < x.getHeight(); ++i) {
        const double err = x.get(0, i);
        const double t = target.get(0, i);
        if (std::abs(err) > std::fmax(std::abs(m_maxError * t), m_minError)) {
            return false;
        }
    }

    return true;
}
