#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_SEIDEL_SLE_SOLVER_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_SEIDEL_SLE_SOLVER_H

#include "sle_solver.h"

namespace atg_scs {
    class GaussSeidelSleSolver : public SleSolver {
        public:
            GaussSeidelSleSolver();
            virtual ~GaussSeidelSleSolver();

            virtual bool solve(
                    SparseMatrix<3> &J,     //3表示每个非零块有3个值,非零块个数为2（默认值）
                    Matrix &W,
                    Matrix &right,
                    Matrix *result,
                    Matrix *previous);
            virtual bool solveWithLimits(
                SparseMatrix<3> &J,
                Matrix &W,
                Matrix &right,
                Matrix &limits,
                Matrix *result,
                Matrix *previous);

            int m_maxIterations;                //最大迭代次数
            double m_minDelta;                  //最小差值

        protected:
            //求解迭代,作为辅助函数,被solve调用
            double solveIteration(
                    Matrix &left,
                    Matrix &right,
                    Matrix *result,
                    Matrix *previous);
            
            //求解迭代,作为辅助函数,被solveWithLimits调用
            double solveIteration(
                    Matrix &left,
                    Matrix &right,
                    Matrix &limits,
                    Matrix *result,
                    Matrix *previous);

            Matrix m_M;                        //M=J^T*W*J
            SparseMatrix<3> m_reg;              //存储中间结果的矩阵
    };
} /* namespace atg_scs */

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_SEIDEL_SLE_SOLVER_H */
