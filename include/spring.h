#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_SPRING_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_SPRING_H

#include "force_generator.h"

#include "rigid_body.h"

namespace atg_scs {

    //弹簧力生成器
    class Spring : public ForceGenerator {
        public:
            Spring();
            virtual ~Spring();

            
            virtual void apply(SystemState *state);
            
            //获取弹簧的两个端点的世界坐标，保存在x_1, y_1, x_2, y_2中
            void getEnds(double *x_1, double *y_1, double *x_2, double *y_2);

            //常量函数不改变对象的成员变量，只返回一个值
            //根据弹簧的静止长度和当前长度计算弹簧的能量
            double energy() const;

            //弹簧的静止长度
            double m_restLength;
            //弹簧的弹性系数
            double m_ks;
            //弹簧的阻尼系数
            double m_kd;

            //弹簧的第一个端点的x坐标（刚体坐标系的本地坐标），端点是弹簧与刚体的连接处
            double m_p1_x;
            double m_p1_y;

            //弹簧的第二个端点的x坐标（刚体坐标系的本地坐标），端点是弹簧与刚体的连接处
            double m_p2_x;
            double m_p2_y;

            //弹簧的第一个端点的刚体
            RigidBody *m_body1;
            //弹簧的第二个端点的刚体
            RigidBody *m_body2;
    };
} /* namespace atg_scs */

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_SPRING_H */
