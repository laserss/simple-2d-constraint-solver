#include "../include/system_state.h"

#include "../include/utilities.h"

#include <assert.h>
#include <cstring>
#include <cmath>

atg_scs::SystemState::SystemState() {
    indexMap = nullptr;

    a_theta = nullptr;
    v_theta = nullptr;
    theta = nullptr;

    a_x = nullptr;
    a_y = nullptr;
    v_x = nullptr;
    v_y = nullptr;
    p_x = nullptr;
    p_y = nullptr;

    f_x = nullptr;
    f_y = nullptr;
    t = nullptr;

    m = nullptr;

    r_x = 0;
    r_y = 0;
    r_t = 0;

    n = 0;
    n_c = 0;
    dt = 0.0;
}

atg_scs::SystemState::~SystemState() {
    //assert是断言，如果条件为假，则程序终止
    assert(n == 0);
    assert(n_c == 0);
}

//将state系统状态复制到当前系统
void atg_scs::SystemState::copy(const SystemState *state) {
    resize(state->n, state->n_c);

    if (state->n == 0) {
        return;
    }

    std::memcpy((void *)indexMap, (void *)state->indexMap, sizeof(int) * n_c);

    std::memcpy((void *)a_theta, (void *)state->a_theta, sizeof(double) * n);
    std::memcpy((void *)v_theta, (void *)state->v_theta, sizeof(double) * n);
    std::memcpy((void *)theta, (void *)state->theta, sizeof(double) * n);

    std::memcpy((void *)a_x, (void *)state->a_x, sizeof(double) * n);
    std::memcpy((void *)a_y, (void *)state->a_y, sizeof(double) * n);
    std::memcpy((void *)v_x, (void *)state->v_x, sizeof(double) * n);
    std::memcpy((void *)v_y, (void *)state->v_y, sizeof(double) * n);
    std::memcpy((void *)p_x, (void *)state->p_x, sizeof(double) * n);
    std::memcpy((void *)p_y, (void *)state->p_y, sizeof(double) * n);

    std::memcpy((void *)f_x, (void *)state->f_x, sizeof(double) * n);
    std::memcpy((void *)f_y, (void *)state->f_y, sizeof(double) * n);
    std::memcpy((void *)t, (void *)state->t, sizeof(double) * n);

    std::memcpy((void *)m, (void *)state->m, sizeof(double) * n);

    std::memcpy((void *)r_x, (void *)state->r_x, sizeof(double) * n_c * 2);
    std::memcpy((void *)r_y, (void *)state->r_y, sizeof(double) * n_c * 2);
    std::memcpy((void *)r_t, (void *)state->r_t, sizeof(double) * n_c * 2);
}


//扩大状态系统的规模至bodyCount或constraintCount，但是系统中的各项数据被清空。
void atg_scs::SystemState::resize(int bodyCount, int constraintCount) {
    if (n >= bodyCount && n_c >= constraintCount) {
        return;
    }

    destroy();

    n = bodyCount;
    n_c = constraintCount;

    indexMap = new int[n_c];

    a_theta = new double[n];
    v_theta = new double[n];
    theta = new double[n];

    a_x = new double[n];
    a_y = new double[n];
    v_x = new double[n];
    v_y = new double[n];
    p_x = new double[n];
    p_y = new double[n];

    f_x = new double[n];
    f_y = new double[n];
    t = new double[n];

    m = new double[n];

    r_x = new double[(size_t)n_c * 2];
    r_y = new double[(size_t)n_c * 2];
    r_t = new double[(size_t)n_c * 2];
}


//将与刚体和约束相关的内存（即system_state中的所有指针）释放，释放后将n和n_c设置为0，dt不修改
void atg_scs::SystemState::destroy() {
    if (n > 0) {
        freeArray(a_theta);
        freeArray(v_theta);
        freeArray(theta);

        freeArray(a_x);
        freeArray(a_y);
        freeArray(v_x);
        freeArray(v_y);
        freeArray(p_x);
        freeArray(p_y);

        freeArray(f_x);
        freeArray(f_y);
        freeArray(t);

        freeArray(m);
    }

    if (n_c > 0) {
        freeArray(indexMap);

        freeArray(r_x);
        freeArray(r_y);
        freeArray(r_t);
    }

    n = 0;
    n_c = 0;
}


//将刚体body的局部坐标(x,y)转换为世界坐标(x_t,y_t)，body为刚体的索引
void atg_scs::SystemState::localToWorld(
        double x,
        double y,
        double *x_t,
        double *y_t,
        int body)
{
    //获取刚体的世界坐标和角度
    const double x0 = p_x[body];
    const double y0 = p_y[body];
    const double theta = this->theta[body];

    const double cos_theta = std::cos(theta);
    const double sin_theta = std::sin(theta);

    *x_t = cos_theta * x - sin_theta * y + x0;
    *y_t = sin_theta * x + cos_theta * y + y0;
}

//计算body刚体上的点（x,y）在世界坐标系中的速度，保存在v_x和v_y中。注意（x,y）是刚体坐标系的本地坐标。
void atg_scs::SystemState::velocityAtPoint(
        double x,
        double y,
        double *v_x,
        double *v_y,
        int body)
{
    //计算点的世界坐标
    double w_x, w_y;
    localToWorld(x, y, &w_x, &w_y, body);

    //获取刚体旋转的角速度
    const double v_theta = this->v_theta[body];
    //角速度转换为线速度
    //算法为：线速度 = 角速度 * (点相对于刚体的距离)
    const double angularToLinear_x = -v_theta * (w_y - this->p_y[body]);
    const double angularToLinear_y = v_theta * (w_x - this->p_x[body]); 

    //线速度 = 刚体本身的线速度 + 本地点（x,y）的线速度
    *v_x = this->v_x[body] + angularToLinear_x;
    *v_y = this->v_y[body] + angularToLinear_y;
}


//对刚体body施加力（f_x,f_y），力作用在刚体坐标系的点（x_l,y_l）上
//影响到this->f_x[body]和this->f_y[body]，以及this->t[body]，分别表示力在x方向和y方向的分量，以及力矩
void atg_scs::SystemState::applyForce(
    double x_l,
    double y_l,
    double f_x,
    double f_y,
    int body)
{
    double w_x, w_y;
    localToWorld(x_l, y_l, &w_x, &w_y, body);

    this->f_x[body] += f_x;
    this->f_y[body] += f_y;

    //计算力矩，力矩 = 力 * 距离
    this->t[body] +=
        (w_y - this->p_y[body]) * -f_x +
        (w_x - this->p_x[body]) * f_y;
}
