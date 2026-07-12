#include "../include/gauss_seidel_sle_solver.h"

#include <cmath>
#include <assert.h>

atg_scs::GaussSeidelSleSolver::GaussSeidelSleSolver()
    : atg_scs::SleSolver(true)
{
    m_maxIterations = 128;
    m_minDelta = 1E-1;

    m_M.initialize(1, 1);
}

atg_scs::GaussSeidelSleSolver::~GaussSeidelSleSolver() {
    m_M.destroy();
    m_reg.destroy();
}

bool atg_scs::GaussSeidelSleSolver::solve(
        SparseMatrix<3> &J,
        Matrix &W,
        Matrix &right,
        Matrix *previous,
        Matrix *result)
{
    const int n = right.getHeight();
    
    result->resize(1, n);   //λ向量

    //使用上一次的λ向量作为初始值（如果存在）
    if (previous != nullptr && previous->getHeight() == n) {
        result->set(previous);
    }

    J.rightScale(W, &m_reg);  //m_reg = J * W
    m_reg.multiplyTranspose(J, &m_M);  //m_M = J^T * W * J

    for (int i = 0; i < m_maxIterations; ++i) {
        const double maxDelta = solveIteration(
                m_M,
                right,
                result,                     //本次调用后被更新为新的*result
                result);           //调用前的*result，即上一次的λ向量

        if (maxDelta < m_minDelta) {
            return true;
        }
    }

    return false;
}

double atg_scs::GaussSeidelSleSolver::solveIteration(
        Matrix &left,
        Matrix &right,
        Matrix *k_next,
        Matrix *k)
{
    double maxDifference = 0.0;
    const int n = k->getHeight();

    //第i次循环，得到k_next(λ)向量中的第i个元素，循环n次后得到新的λ向量即k_next
    for (int i = 0; i < n; ++i) {
        double s0 = 0.0, s1 = 0.0;
        for (int j = 0; j < i; ++j) {
            s0 += left.get(j, i) * k_next->get(0, j);
        }

        for (int j = i + 1; j < n; ++j) {
            s1 += left.get(j, i) * k->get(0, j);
        }

        const double k_next_i =
            (1 / left.get(i, i)) * (right.get(0, i) - s0 - s1);

        const double min_k = std::fmax(1E-3, k->get(0, i));
        const double delta = (std::abs(k_next_i) - min_k) / min_k;
        maxDifference = (delta > maxDifference)
            ? delta
            : maxDifference;

        k_next->set(0, i, k_next_i);
    }

    return maxDifference;
}

bool atg_scs::GaussSeidelSleSolver::solveWithLimits(
    SparseMatrix<3> &J,
    Matrix &W,
    Matrix &right,
    Matrix &limits,
    Matrix *result,
    Matrix *previous)
{
    const int n = right.getHeight();
    if (result->getHeight() != n) {
        result->initialize(1, n);
    }

    if (previous != nullptr && previous->getHeight() == n) {
        result->set(previous);
    }

    J.rightScale(W, &m_reg);
    m_reg.multiplyTranspose(J, &m_M);

    for (int i = 0; i < m_maxIterations; ++i) {
        const double maxDelta = solveIteration(
            m_M,
            right,
            limits,
            result,
            result);

        if (maxDelta < m_minDelta) {
            return true;
        }
    }

    return false;
}

double atg_scs::GaussSeidelSleSolver::solveIteration(
        Matrix &left,
        Matrix &right,
        Matrix &limits,
        Matrix *k_next,
        Matrix *k)
{
    double maxDifference = 0.0;
    const int n = k->getHeight();

    for (int i = 0; i < n; ++i) {
        double s0 = 0.0, s1 = 0.0;
        for (int j = 0; j < i; ++j) {
            s0 += left.get(j, i) * k_next->get(0, j);
        }

        for (int j = i + 1; j < n; ++j) {
            s1 += left.get(j, i) * k->get(0, j);
        }

        const double k_next_i =
            (1 / left.get(i, i)) * (right.get(0, i) - s0 - s1);

        //限制λi的取值范围，防止λi过大或过小
        const double limitMin = limits.get(0, i);
        const double limitMax = limits.get(1, i);
        const double x = std::fmax(limitMin, std::fmin(limitMax, k_next_i));

        const double min_k = std::fmax(1E-3, std::abs(k->get(0, i)));
        const double delta = std::abs(x - k->get(0, i)) / min_k;
        maxDifference = (delta > maxDifference)
            ? delta
            : maxDifference;

        k_next->set(0, i, x);
    }

    return maxDifference;
}
