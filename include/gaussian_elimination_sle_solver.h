#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_GAUSSIAN_ELIMINATION_SLE_SOLVER_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_GAUSSIAN_ELIMINATION_SLE_SOLVER_H

#include "sle_solver.h"

#include "utilities.h"

namespace atg_scs {
    class GaussianEliminationSleSolver : public SleSolver {
        public:
            GaussianEliminationSleSolver();
            virtual ~GaussianEliminationSleSolver();

            virtual bool solve(
                    SparseMatrix<3> &J,
                    Matrix &W,
                    Matrix &right,
                    Matrix *result,
                    Matrix *previous);

            static scs_force_inline double fastAbs(double v) {
                return (v > 0)
                    ? v
                    : -v;
            }

        protected:
            Matrix m_a; //增广矩阵:[系数矩阵 | 右侧项],即 [m_M | right]
            Matrix m_M; //系数矩阵
            SparseMatrix<3> m_reg;
    };
} /* namespace atg_scs */

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_GAUSSIAN_ELIMINATION_SLE_SOLVER_H */
