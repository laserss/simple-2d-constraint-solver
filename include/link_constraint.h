#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_LINK_CONSTRAINT_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_LINK_CONSTRAINT_H

#include "constraint.h"

namespace atg_scs {
    class LinkConstraint : public Constraint {
        public:
            LinkConstraint();
            virtual ~LinkConstraint();
            
            void setBody1(RigidBody *body) { m_bodies[0] = body; }
            void setBody2(RigidBody *body) { m_bodies[1] = body; }

            //设置刚体1与约束连接点的局部坐标
            void setLocalPosition1(double x, double y);
            //设置刚体2与约束连接点的局部坐标
            void setLocalPosition2(double x, double y);

            virtual void calculate(Output *output, SystemState *system);

            double m_maxForce;

            //刚体1连接点的局部坐标x
            double m_local_x_1;
            double m_local_y_1;
            //刚体2连接点的局部坐标x
            double m_local_x_2;
            double m_local_y_2;

            double m_ks;    //stiffness 弹性系数
            double m_kd;    //damping 阻尼系数
    };
} /* namespace atg_scs */

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_LINK_CONSTRAINT_H */
