#include "../include/link_constraint.h"

#include <cmath>

//约束数量为2表示只有x方向和y方向的约束
atg_scs::LinkConstraint::LinkConstraint() : Constraint(2, 2) {
    m_local_x_1 = m_local_y_1 = 0.0;
    m_local_x_2 = m_local_y_2 = 0.0;
    m_ks = 10.0;
    m_kd = 1.0;
    m_maxForce = DBL_MAX;
}

atg_scs::LinkConstraint::~LinkConstraint() {
    /* void */
}

void atg_scs::LinkConstraint::calculate(
        Output *output,
        SystemState *state)
{
    //刚体1
    const int body = m_bodies[0]->index;
    //与刚体1连接的刚体2
    const int linkedBody = m_bodies[1]->index;

    //刚体1的世界坐标x
    const double q1 = state->p_x[body];
    //刚体1的世界坐标y
    const double q2 = state->p_y[body];
    //刚体1的角度
    const double q3 = state->theta[body];

    //刚体2的世界坐标x
    const double q4 = state->p_x[linkedBody];
    //刚体2的世界坐标y
    const double q5 = state->p_y[linkedBody];
    //刚体2的角度
    const double q6 = state->theta[linkedBody];

    //刚体1角度求导，即角速度
    const double q3_dot = state->v_theta[body];
    //刚体2角度求导，即角速度
    const double q6_dot = state->v_theta[linkedBody];

    const double cos_q3 = std::cos(q3);
    const double sin_q3 = std::sin(q3);

    const double cos_q6 = std::cos(q6);
    const double sin_q6 = std::sin(q6);

    //刚体1与约束连接点的世界坐标
    const double bodyX = q1 + cos_q3 * m_local_x_1 - sin_q3 * m_local_y_1;
    const double bodyY = q2 + sin_q3 * m_local_x_1 + cos_q3 * m_local_y_1;

    //刚体2与约束连接点的世界坐标
    const double linkedBodyX = q4 + cos_q6 * m_local_x_2 - sin_q6 * m_local_y_2;
    const double linkedBodyY = q5 + sin_q6 * m_local_x_2 + cos_q6 * m_local_y_2;

    //两个连接点之间的坐标差值，如果C1、C2都为0，则表示两个刚体被一个“合页”或“铰链”连接
    const double C1 = bodyX - linkedBodyX;
    const double C2 = bodyY - linkedBodyY;


    //雅可比矩阵J(2行6列,J矩阵最大支持3行六列，此处只用到了2行6列，因为不涉及角度的约束)，表示约束C1、C2对刚体1、刚体2的各个状态变量(世界坐标x、y,角度theta)的偏导数
    //每一行对应一个约束方程，每一列对应一个自由度。物理含义是：当某个自由度变化时，约束违反量会如何变化。
    //雅可比矩阵的每一行表示一个约束方程对各个自由度的偏导数，每一列表示一个自由度对各个约束方程的偏导数。
    //C1对刚体1世界坐标x(即q1)的偏导数
    output->J[0][0] = 1.0;
    //C1对刚体1世界坐标y(即q2)的偏导数,C1是x方向的约束，所以对y的偏导数为0
    output->J[0][1] = 0.0;
    //C1对刚体1角度(即q3)的偏导数，如果连接点在刚体1的中心，则m_local_x_1和m_local_y_1为0，则偏导数为0
    output->J[0][2] = -sin_q3 * m_local_x_1 - cos_q3 * m_local_y_1;

    //C2对刚体1世界坐标x(即q1)的偏导数
    output->J[1][0] = 0.0;
    //C2对刚体1世界坐标y(即q2)的偏导数,C2是y方向的约束，所以对x的偏导数为0
    output->J[1][1] = 1.0;
    //C2对刚体1角度(即q3)的偏导数，如果连接点在刚体1的中心，则m_local_x_1和m_local_y_1为0，则偏导数为0
    output->J[1][2] = cos_q3 * m_local_x_1 - sin_q3 * m_local_y_1;

    //C1对刚体2世界坐标x(即q4)的偏导数
    output->J[0][3] = -1.0;
    //C1对刚体2世界坐标y(即q5)的偏导数
    output->J[0][4] = 0.0;
    //C1对刚体2角度(即q6)的偏导数
    output->J[0][5] = sin_q6 * m_local_x_2 + cos_q6 * m_local_y_2;

    //C2对刚体2世界坐标x(即q4)的偏导数
    output->J[1][3] = 0.0;
    //C2对刚体2世界坐标y(即q5)的偏导数
    output->J[1][4] = -1.0;
    //C2对刚体2角度(即q6)的偏导数
    output->J[1][5] = -cos_q6 * m_local_x_2 + sin_q6 * m_local_y_2;


    //雅可比矩阵J的导数（J对时间求导）J_dot
    output->J_dot[0][0] = 0;
    output->J_dot[0][1] = 0;
    //C1对刚体1角度(即q3)的偏导数对时间求导
    output->J_dot[0][2] =
        -cos_q3 * q3_dot * m_local_x_1 + sin_q3 * q3_dot * m_local_y_1;

    output->J_dot[1][0] = 0;
    output->J_dot[1][1] = 0;
    //C2对刚体1角度(即q3)的偏导数对时间求导
    output->J_dot[1][2] =
        -sin_q3 * q3_dot * m_local_x_1 - cos_q3 * q3_dot * m_local_y_1;

    output->J_dot[0][3] = 0;
    output->J_dot[0][4] = 0;
    //C1对刚体2角度(即q6)的偏导数对时间求导
    output->J_dot[0][5] =   
        cos_q6 * q6_dot * m_local_x_2 - sin_q6 * q6_dot * m_local_y_2;

    output->J_dot[1][3] = 0;
    output->J_dot[1][4] = 0;
    //C2对刚体2角度(即q6)的偏导数对时间求导
    output->J_dot[1][5] =   
        sin_q6 * q6_dot * m_local_x_2 + cos_q6 * q6_dot * m_local_y_2;

    output->kd[0] = m_kd;
    output->kd[1] = m_kd;
    output->ks[0] = m_ks;
    output->ks[1] = m_ks;

    output->C[0] = C1;
    output->C[1] = C2;

    output->v_bias[0] = 0;
    output->v_bias[1] = 0;

    output->limits[0][0] = -m_maxForce;
    output->limits[0][1] = m_maxForce;

    output->limits[1][0] = -m_maxForce;
    output->limits[1][1] = m_maxForce;
}

//设置刚体1与约束连接点的局部坐标
void atg_scs::LinkConstraint::setLocalPosition1(double x, double y) {
    m_local_x_1 = x;
    m_local_y_1 = y;
}

//设置刚体2与约束连接点的局部坐标
void atg_scs::LinkConstraint::setLocalPosition2(double x, double y) {
    m_local_x_2 = x;
    m_local_y_2 = y;
}
