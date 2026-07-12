#include "../include/gaussian_elimination_sle_solver.h"

#include <cmath>
#include <assert.h>
#include <fstream>

// 高斯消元线性方程组求解器。
//
// 在约束系统中，约束力通过拉格朗日乘子 λ 表示。
// GenericRigidBodySystem 会先构造如下形式的线性方程：
//
//     J · W · Jᵀ · λ = right
//
// 其中：
//   J     : 约束雅可比矩阵，每一行对应一个约束方程；
//   W     : 质量矩阵 M 的逆矩阵，这里以对角列向量形式存储；
//   right : 约束方程右侧项，包含 J_dot、外力、位置误差、速度误差等影响；
//   λ     : 要求解的拉格朗日乘子，后续用于计算约束反力 R = Jᵀ · λ。
//
// 本类使用直接法（Gaussian elimination）求解 λ。
// 该求解器不支持带上下限的约束求解，因此基类 SleSolver(false)。
atg_scs::GaussianEliminationSleSolver::GaussianEliminationSleSolver()
    : atg_scs::SleSolver(false)
{
    // 预初始化临时矩阵，后续 solve() 会按实际约束数量 resize。
    // m_a: 增广矩阵 [A | b]，A = J · W · Jᵀ，b = right。
    // m_M: 系数矩阵 A。
    m_a.initialize(1, 1);
    m_M.initialize(1, 1);
}

atg_scs::GaussianEliminationSleSolver::~GaussianEliminationSleSolver() {
    // 释放内部缓存。Matrix/SparseMatrix 的析构函数会断言资源已释放，
    // 所以这里显式 destroy()。
    m_a.destroy();
    m_M.destroy();
    m_reg.destroy();
}

//？previous 与 result 顺序与头文件中定义不一致，作者是否写错？
// 输入参数：J,W,right,previous(未使用),输出result（λ）
bool atg_scs::GaussianEliminationSleSolver::solve(
        SparseMatrix<3> &J,
        Matrix &W,
        Matrix &right,
        Matrix *previous,           //此函数没有使用previous,列出只是为了兼容基类
        Matrix *result)             //result就是λ，一个列向量
{
    // 计算系数矩阵：
    //
    //     A = J · W · Jᵀ
    //
    // 先把 J 的每个广义坐标列按 W 中的逆质量/逆转动惯量缩放，
    // 得到 m_reg = J · W；再乘以 Jᵀ 得到密集矩阵 m_M。
    J.rightScale(W, &m_reg);
    m_reg.multiplyTranspose(J, &m_M);

    // m_M 的尺寸是 m × m，m 为约束方程数量。
    // 增广矩阵 A 需要多一列保存 right，所以宽度 n = m + 1。
    //
    // Matrix 使用 get(column, row) / set(column, row) 访问，
    // 因此这里 n 表示列数，m 表示行数。
    const int n = m_M.getWidth() + 1;
    const int m = m_M.getHeight();

    // right 必须是高度为 m 的列向量，每个约束方程对应一个右侧值。
    assert(right.getHeight() == m_M.getWidth());

    // 无约束时没有方程需要求解，直接视为成功。
    if (n == 0 || m == 0) return true;

    // result 保存 λ，是高度为 m 的列向量。
    result->resize(1, m);

    Matrix &A = m_a;
    A.resize(n, m);

    // 构造增广矩阵：
    //
    //     [ m_M | right ]
    //
    // A 的前 m 列为系数矩阵，最后一列为右侧项。
    for (int i = 0; i < m; ++i) {
        //处理前（n-1)列，即系数矩阵
        for (int j = 0; j < n - 1; ++j) {
            A.set(j, i, m_M.get(j, i));
        }
        //处理第n列，即右侧项
        A.set(n - 1, i, right.get(0, i));
    }

    // 前向消元，将增广矩阵化为上三角形式。
    //
    // h: 当前主元所在行；
    // k: 当前主元所在列。
    //
    // 每一轮在当前列 k 的 h..m-1 行中选绝对值最大的元素作为主元，
    // 这是“部分主元选择”，可以降低除以很小数导致的数值误差。
    int h = 0, k = 0;
    while (h < m && k < n) {
        int i_max = h;
        double maxV = fastAbs(A.get(k, i_max));
        for (int i = h + 1; i < m; ++i) {
            const double v = fastAbs(A.get(k, i));
            if (v > maxV) {
                maxV = v;
                i_max = i;
            }
        }

        if (maxV == 0) {
            // 当前列没有可用主元，说明这一列在剩余方程中全为 0。
            // 跳过该列，继续寻找下一列主元。
            ++k;
        }
        else {
            // 把主元行交换到当前行 h。
            A.fastRowSwap(h, i_max);

            // 用主元行消去下方各行在第 k 列上的值。
            for (int i = h + 1; i < m; ++i) {
                const double f = A.get(k, i) / A.get(k, h);
                A.set(k, i, 0.0);

                // 从 k+1 列开始更新即可；第 k 列已经被显式设置为 0。
                for (int j = k + 1; j < n; ++j) {
                    A.set(j, i, A.get(j, i) - A.get(j, h) * f);
                }
            }

            ++h;
            ++k;
        }
    }

    // 最后一行最后一个系数为 0 时，无法按当前回代逻辑求解最后一个未知数。
    // 这通常意味着系数矩阵奇异或约束方程线性相关。
    if (A.get(n - 2, m - 1) == 0) {
        assert(false);
    }

    // 回代求解 λ。
    //
    // 上三角矩阵最后一行只有最后一个未知数，先求 x_m，
    // 然后从倒数第二行往上逐个求解。
    const double x_m = A.get(n - 1, m - 1) / A.get(n - 2, m - 1);
    //result就是λ，一个列向量
    result->set(0, m - 1, x_m);
    for (int i = m - 2; i >= 0; --i) {
        const double b_i = A.get(n - 1, i);
        double sum = 0.0;
        for (int j = m - 1; j > i; --j) {
            sum += A.get(j, i) * result->get(0, j);
        }

        if (A.get(i, i) != 0) {
            // x_i = (b_i - Σ a_ij x_j) / a_ii
            result->set(0, i, (b_i - sum) / A.get(i, i));
        }
        else {
            // 对退化行给出 0。这里没有返回失败，保持原实现行为。
            result->set(0, i, 0);
        }
    }

    // 数值安全检查：λ(result)中不能出现 NaN 或无穷大。
    for (int i = 0; i < m; ++i) {
        if (std::isnan(result->get(0, i)) || std::isinf(result->get(0, i))) {
            assert(false);
        }
    }

    // 只要没有触发断言，直接法求解完成。
    return true;
}
