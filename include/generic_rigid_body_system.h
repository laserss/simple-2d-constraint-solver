#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_GENERIC_RIGID_BODY_SYSTEM_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_GENERIC_RIGID_BODY_SYSTEM_H

#include "rigid_body_system.h"

#include "sle_solver.h"
#include "ode_solver.h"

namespace atg_scs {
    class GenericRigidBodySystem : public RigidBodySystem {
        public:
            GenericRigidBodySystem();
            //此处使用虚函数的原因是：？？？
            virtual ~GenericRigidBodySystem();

            void initialize(SleSolver *sleSolver, OdeSolver *odeSolver);
            virtual void process(double dt, int steps = 1);

        protected:
            void processConstraints(
                    long long *evalTime,
                    long long *solveTime);

        protected:
            OdeSolver *m_odeSolver;//OdeSolver 算积分 q（位置）, v（速度）
            SleSolver *m_sleSolver;//SleSolver 求 λ的解算器

        protected:
            //中间值结构体，用于存储中间计算结果
            struct IntermediateValues {
                //sreg0：稀疏临时寄存器（s = sparse，reg = register）
                //来源：m_iv.J_sparse.leftScale(m_iv.lambda, &m_iv.sreg0);  (λ·J)ᵀ=sreg0
                SparseMatrix<3> J_sparse, J_dot_sparse, sreg0;
                Matrix J_T;
                Matrix M, M_inv;        //M矩阵和M_inv都是对角矩阵为什么不使用稀疏矩阵存储？因为  在RigidBodySystem::populateMassMatrices中直接将M和_inv设置为列向量了。可以比稀疏矩阵更进一步的压缩。
                Matrix C;
                Matrix ks, kd;
                Matrix q_dot;

                Matrix reg0, reg1, reg2;

                Matrix right;
                Matrix F_ext, F_C, R;

                // 为什么将矩阵命名为lambda？lambda就是λ，拉格朗日乘子
                Matrix lambda;
            } m_iv;
    };
} /* namespace atg_scs */

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_GENERIC_RIGID_BODY_SYSTEM_H */
