#include "../include/spring.h"

#include <cmath>

atg_scs::Spring::Spring() {
    m_restLength = 1.0;
    m_ks = 0;
    m_kd = 0;

    m_p1_x = m_p1_y = 0;
    m_p2_x = m_p2_y = 0;

    m_body1 = m_body2 = nullptr;
}

atg_scs::Spring::~Spring() {
    /* void */
}

void atg_scs::Spring::apply(SystemState *state) {
    //任意一个端点为空，则不应用弹簧力
    if (m_body1 == nullptr || m_body2 == nullptr) return;

    //两个端点的世界坐标
    double x1, y1;
    double x2, y2;

    //两个端点的全局速度
    double v_x1 = 0, v_y1 = 0;
    double v_x2 = 0, v_y2 = 0;

    if (m_body1->index != -1) { 
        //若刚体在刚体系统中，使用state的localToWorld函数通过state中记录的刚体世界坐标及角度，将端点坐标转换为世界坐标
        state->localToWorld(m_p1_x, m_p1_y, &x1, &y1, m_body1->index);

        //计算第一个端点的全局速度
        state->velocityAtPoint(m_p1_x, m_p1_y, &v_x1, &v_y1, m_body1->index);
    }
    else {  //若刚体不在刚体系统中，则使用刚体自身的localToWorld函数将端点坐标转换为世界坐标
        m_body1->localToWorld(m_p1_x, m_p1_y, &x1, &y1);
    }

    if (m_body2->index != -1) {
        state->localToWorld(m_p2_x, m_p2_y, &x2, &y2, m_body2->index);
        state->velocityAtPoint(m_p2_x, m_p2_y, &v_x2, &v_y2, m_body2->index);
    }
    else {
        m_body2->localToWorld(m_p2_x, m_p2_y, &x2, &y2);
    }

    //计算两个端点的距离 l
    double dx = x2 - x1;
    double dy = y2 - y1;
    const double l = std::sqrt(dx * dx + dy * dy);

////////////////////////////////////////////////////////////////////////save point, start here////////////////////////////////////////////////////////////////////////
    //如果距离l大于0.01，则将向量（dx,dy）归一化。归一化之后(dx,dy)是从端点1指向端点2的单位方向向量
    if (std::abs(l) >= 1E-2) {
        dx /= l;    //dx = dx / l;
        dy /= l;    //dy = dy / l;
    }
    else {
        dx = 0.0;
        dy = 0.0;
    }

    //计算两个端点的相对速度
    const double rel_v_x = (v_x2 - v_x1);
    const double rel_v_y = (v_y2 - v_y1);

    //v=(dx,dy)*(rel_v_x,rel_v_y)=dx*rel_v_x + dy*rel_v_y，表示相对速度在弹簧方向上的投影，即两端点沿弹簧轴方向相互靠近或远离的速度
    //atg_scs::Spring::apply函数后续未使用v，可能是为了后续扩展使用
    const double v = dx * rel_v_x + dy * rel_v_y;
    //x表示弹簧的当前长度与静止长度之差，即弹簧的伸长量或压缩量
    const double x = l - m_restLength;

    //对刚体1施加力，力的大小为dx * x * m_ks + rel_v_x * m_kd，力的方向为(dx,dy)，力的作用点为(m_p1_x,m_p1_y)


    
    state->applyForce(
        m_p1_x,
        m_p1_y,
        dx * x * m_ks + rel_v_x * m_kd,
        dy * x * m_ks + rel_v_y * m_kd,
        m_body1->index
    );

    state->applyForce(
        m_p2_x,
        m_p2_y,
        -dx * x * m_ks - rel_v_x * m_kd,
        -dy * x * m_ks - rel_v_y * m_kd,
        m_body2->index
    );
}

//获取弹簧的两个端点的世界坐标
void atg_scs::Spring::getEnds(double *x_1, double *y_1, double *x_2, double *y_2) {
    if (m_body1 == nullptr || m_body2 == nullptr) return;

    m_body1->localToWorld(m_p1_x, m_p1_y, x_1, y_1);
    m_body2->localToWorld(m_p2_x, m_p2_y, x_2, y_2);
}

//根据弹簧的静止长度和当前长度计算弹簧的能量
//能量 = 0.5 * 弹性系数 * (当前长度 - 静止长度) * (当前长度 - 静止长度)
double atg_scs::Spring::energy() const {
    if (m_body1 == nullptr || m_body2 == nullptr) return 0;

    double x1, y1;
    double x2, y2;

    m_body1->localToWorld(m_p1_x, m_p1_y, &x1, &y1);
    m_body2->localToWorld(m_p2_x, m_p2_y, &x2, &y2);

    const double dx = x2 - x1;
    const double dy = y2 - y1;

    const double l = std::sqrt(dx * dx + dy * dy);

    return 0.5 * m_ks * (l - m_restLength) * (l - m_restLength);
}
