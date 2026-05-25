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
            //??????????最大约束数量为什么限制为3？（x方向、y方向、角度差值?）
            //??????????最大刚体数量为什么限制为2？ (刚体1、刚体2,一个约束连接两个刚体)
            static constexpr int MaxConstraintCount = 3;
            static constexpr int MaxBodyCount = 2;

            struct Output {
                //储存连接点之间的坐标差值，分为x方向和y方向及角度差值（是否有角度差值？）
                double C[MaxConstraintCount];
                //约束的雅可比矩阵
                double J[MaxConstraintCount][3 * MaxBodyCount];
                //约束的雅可比矩阵的导数（J对时间求导）
                double J_dot[MaxConstraintCount][3 * MaxBodyCount];
                //?约束的变量偏差（v_bias）是约束方程的右侧常数项，表示约束被违反的程度。
                //?在求解约束时，需要将v_bias项移到等式左侧，以便求解约束力。
                double v_bias[MaxConstraintCount];
                //?约束的变量限制（limits）是约束方程的左侧常数项，表示约束被违反的程度。
                //?在求解约束时，需要将limits项移到等式左侧，以便求解约束力。
                //数组大小设置为2，用于存放约束力的下限和上限，例如limits[0][0]表示x方向的约束下限，limits[0][1]表示x方向的约束上限
                double limits[MaxConstraintCount][2];
                //?约束的弹簧刚度（ks）是约束方程的左侧常数项，表示约束被违反的程度。
                //?在求解约束时，需要将ks项移到等式左侧，以便求解约束力。
                double ks[MaxConstraintCount];
                //?约束的阻尼刚度（kd）是约束方程的左侧常数项，表示约束被违反的程度。
                //?在求解约束时，需要将kd项移到等式左侧，以便求解约束力。
                double kd[MaxConstraintCount];
            };

        public:
            Constraint(int constraintCount, int bodyCount);
            virtual ~Constraint();

            virtual void calculate(Output *output, SystemState *state);

            //普通函数调用需要：压栈 → 跳转 → 执行 → 返回 → 出栈，这些开销比函数体本身还大。内联后编译器会直接把调用处替换为 m_constraintCount，完全消除函数调用开销。
            scs_force_inline int getConstraintCount() const { return m_constraintCount; }

            //约束的索引？
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
            //约束的数量(如果设置为3表示有x方向、y方向、角度三个约束)
            int m_constraintCount;
    };
} /* namespace atg_scs */

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_CONSTRAINT_H */
