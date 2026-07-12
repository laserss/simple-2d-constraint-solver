#include "../include/rigid_body_system.h"

#include <assert.h>
#include <chrono>
#include <cmath>

atg_scs::RigidBodySystem::RigidBodySystem() {
    m_odeSolveMicroseconds = new long long[ProfilingSamples];
    m_constraintSolveMicroseconds = new long long[ProfilingSamples];
    m_forceEvalMicroseconds = new long long[ProfilingSamples];
    m_constraintEvalMicroseconds = new long long[ProfilingSamples];
    m_frameIndex = 0;

    for (int i = 0; i < ProfilingSamples; ++i) {
        m_odeSolveMicroseconds[i] = -1;
        m_constraintSolveMicroseconds[i] = -1;
        m_forceEvalMicroseconds[i] = -1;
        m_constraintEvalMicroseconds[i] = -1;
    }
}

atg_scs::RigidBodySystem::~RigidBodySystem() {
    delete[] m_odeSolveMicroseconds;
    delete[] m_constraintSolveMicroseconds;
    delete[] m_forceEvalMicroseconds;
    delete[] m_constraintEvalMicroseconds;

    m_state.destroy();
}

void atg_scs::RigidBodySystem::reset() {
    m_rigidBodies.clear();
    m_constraints.clear();
    m_forceGenerators.clear();
}

void atg_scs::RigidBodySystem::process(double dt, int steps) {
    /* void */
}

void atg_scs::RigidBodySystem::addRigidBody(RigidBody *body) {
    m_rigidBodies.push_back(body);
    body->index = (int)m_rigidBodies.size() - 1;
}

void atg_scs::RigidBodySystem::removeRigidBody(RigidBody *body) {
    m_rigidBodies[body->index] = m_rigidBodies.back();
    m_rigidBodies[body->index]->index = body->index;
    m_rigidBodies.resize(m_rigidBodies.size() - 1);
}

atg_scs::RigidBody *atg_scs::RigidBodySystem::getRigidBody(int i) {
    assert(i < m_rigidBodies.size());
    return m_rigidBodies[i];
}

void atg_scs::RigidBodySystem::addConstraint(Constraint *constraint) {
    m_constraints.push_back(constraint);
    constraint->m_index = (int)m_constraints.size() - 1;
}

void atg_scs::RigidBodySystem::removeConstraint(Constraint *constraint) {
    m_constraints[constraint->m_index] = m_constraints.back();
    m_constraints[constraint->m_index]->m_index = constraint->m_index;
    m_constraints.resize(m_constraints.size() - 1);
}

void atg_scs::RigidBodySystem::addForceGenerator(ForceGenerator *forceGenerator) {
    m_forceGenerators.push_back(forceGenerator);
    forceGenerator->m_index = (int)m_forceGenerators.size() - 1;
}

void atg_scs::RigidBodySystem::removeForceGenerator(ForceGenerator *forceGenerator) {
    m_forceGenerators[forceGenerator->m_index] = m_forceGenerators.back();
    m_forceGenerators[forceGenerator->m_index]->m_index = forceGenerator->m_index;
    m_forceGenerators.resize(m_forceGenerators.size() - 1);
}

//累加RigidBodySystem中每一个约束实例包含的约束数量(如LinkConstraint包含x和y方向2个约束，则累加2)
int atg_scs::RigidBodySystem::getFullConstraintCount() const {
    int count = 0;
    for (Constraint *constraint: m_constraints) {
        count += constraint->getConstraintCount();
    }

    return count;
}
//计算samples数组中所有非-1的值的平均值,samples数组中存储了600个时间步长的性能数据
float atg_scs::RigidBodySystem::findAverage(long long *samples) {
    long long accum = 0;
    int count = 0;
    //ProfilingSamples的值为600
    for (int i = 0; i < ProfilingSamples; ++i) {
        if (samples[i] != -1) {
            accum += samples[i];
            ++count;
        }
    }

    if (count == 0) return 0;
    else return (float)accum / count;
}

float atg_scs::RigidBodySystem::getOdeSolveMicroseconds() const {
    return findAverage(m_odeSolveMicroseconds);
}

float atg_scs::RigidBodySystem::getConstraintSolveMicroseconds() const {
    return findAverage(m_constraintSolveMicroseconds);
}

float atg_scs::RigidBodySystem::getConstraintEvalMicroseconds() const {
    return findAverage(m_constraintEvalMicroseconds);
}

float atg_scs::RigidBodySystem::getForceEvalMicroseconds() const {
    return findAverage(m_forceEvalMicroseconds);
}

//填充系统状态，将刚体和约束的状态填充到系统状态中
void atg_scs::RigidBodySystem::populateSystemState() {
    const int n = getRigidBodyCount();
    //累加RigidBodySystem中每一个约束实例包含的约束数量(如LinkConstraint包含x和y方向2个约束，则累加2)
    const int n_c = getFullConstraintCount();
    //获取RigidBodySystem中约束容器m_constraints中约束实例的数量
    const int m = getConstraintCount();

    m_state.resize(n, n_c);

    //把每一个刚体的信息填充到系统状态中
    for (int i = 0; i < n; ++i) {
        m_state.a_x[i] = 0;
        m_state.a_y[i] = 0;

        m_state.v_x[i] = m_rigidBodies[i]->v_x;
        m_state.v_y[i] = m_rigidBodies[i]->v_y;

        m_state.p_x[i] = m_rigidBodies[i]->p_x;
        m_state.p_y[i] = m_rigidBodies[i]->p_y;

        m_state.a_theta[i] = 0;
        m_state.v_theta[i] = m_rigidBodies[i]->v_theta;
        m_state.theta[i] = m_rigidBodies[i]->theta;

        m_state.m[i] = m_rigidBodies[i]->m;
    }
    /*  indexMap 循环在做什么？ 记录每一个约束对象在全局展平的约束方程数组中的起始位置。
    
        关键前提：一个 Constraint 对象可以包含多个标量约束方程（如 LinkConstraint 同时约束 x 和 y，有 2 个方程）。
        indexMap[i] 记录的是：第 i 个约束对象，在全局展平的约束方程数组中，从第几行开始。
        举例，假设有 3 个约束：
        约束对象 0（LinkConstraint）：  getConstraintCount() = 2
        约束对象 1（FixedRotation）：   getConstraintCount() = 1
        约束对象 2（LinkConstraint）：  getConstraintCount() = 2
        循环执行后：
        indexMap[0] = 0    ← 约束0 的方程从 J 的第 0 行开始
        indexMap[1] = 2    ← 约束1 的方程从 J 的第 2 行开始
        indexMap[2] = 3    ← 约束2 的方程从 J 的第 3 行开始
        本质上是对 getConstraintCount() 做前缀和（prefix sum）：
        indexMap[i] = Σ(k=0 to i-1) constraints[k]->getConstraintCount()
        这样当代码需要操作某个约束对象对应的 J 矩阵行、lambda 值、或约束反力 r_x/r_y/r_t 时，能直接通过 indexMap[i] 跳转到正确的起始位置，而不用每次都从头累加。
    */
    for (int i = 0, j_f = 0; i < m; ++i) {
        m_state.indexMap[i] = j_f;
        j_f += m_constraints[i]->getConstraintCount();
    }
}

//向列向量M、M_inv添加各个刚体的参数
void atg_scs::RigidBodySystem::populateMassMatrices(Matrix *M, Matrix *M_inv) {
    const int n = getRigidBodyCount();

    M->initialize(1, 3 * n);
    M_inv->initialize(1, 3 * n);

    for (int i = 0; i < n; ++i) {
        M->set(0, i * 3 + 0, m_rigidBodies[i]->m);
        M->set(0, i * 3 + 1, m_rigidBodies[i]->m);
        M->set(0, i * 3 + 2, m_rigidBodies[i]->I);

        M_inv->set(0, i * 3 + 0, 1 / m_rigidBodies[i]->m);
        M_inv->set(0, i * 3 + 1, 1 / m_rigidBodies[i]->m);
        M_inv->set(0, i * 3 + 2, 1 / m_rigidBodies[i]->I);
     }
}

//计算所有力生成器对刚体的作用力
void atg_scs::RigidBodySystem::processForces() {
    const int n_f = getForceGeneratorCount();
    const int n = getRigidBodyCount();

    //初始化刚体的受力为0
    for (int i = 0; i < n; ++i) {
        m_state.f_x[i] = 0.0;
        m_state.f_y[i] = 0.0;
        m_state.t[i] = 0.0;
    }

    for (int i = 0; i < n_f; ++i) {
        //计算力生成器对刚体的作用力
        m_forceGenerators[i]->apply(&m_state);
    }
}
