#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_CONSTRAINT_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_CONSTRAINT_H

#include "system_state.h"
#include "rigid_body.h"
#include "matrix.h"
#include "utilities.h"
//""来自项目文件夹，<>来自系统库
#include <cfloat>

namespace atg_scs {
    class Constraint {
        public:
            //static（在类内成员里）：表示这是类级别共享成员，不属于某个对象实例。
            //constexpr（在类内成员里）：表示这是编译时常量，在编译时计算。
            //static constexpr int MaxConstraintCount = 3; 这是一个“类共享的、编译期已确定、不可修改”的整型常量，值为 3。
            //??????????最大约束数量为什么限制为3？
            //??????????最大刚体数量为什么限制为2？
            static constexpr int MaxConstraintCount = 3;
            static constexpr int MaxBodyCount = 2;

            struct Output {
                double C[MaxConstraintCount];
                double J[MaxConstraintCount][3 * MaxBodyCount];
                double J_dot[MaxConstraintCount][3 * MaxBodyCount];
                double v_bias[MaxConstraintCount];
                double limits[MaxConstraintCount][2];
                double ks[MaxConstraintCount];
                double kd[MaxConstraintCount];
            };

        public:
            Constraint(int constraintCount, int bodyCount);
            virtual ~Constraint();

            virtual void calculate(Output *output, SystemState *state);
            scs_force_inline int getConstraintCount() const { return m_constraintCount; }

            int m_index;
            int m_bodyCount;
            RigidBody *m_bodies[MaxBodyCount];

            double F_x[MaxConstraintCount][MaxBodyCount];
            double F_y[MaxConstraintCount][MaxBodyCount];
            double F_t[MaxConstraintCount][MaxBodyCount];

        protected:
            inline void noLimits(Output *output) {
                for (int i = 0; i < MaxConstraintCount; ++i) {
                    output->limits[i][0] = -DBL_MAX;
                    output->limits[i][1] = DBL_MAX;
                }
            }

        protected:
            int m_constraintCount;
    };
} /* namespace atg_scs */

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_CONSTRAINT_H */
