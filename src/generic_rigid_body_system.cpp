#include "../include/generic_rigid_body_system.h"

//用于时间测量，在process()函数中使用，记录每个步骤的时间消耗。
#include <chrono>


atg_scs::GenericRigidBodySystem::GenericRigidBodySystem() {
    m_sleSolver = nullptr;//SleSolver 求 λ的解算器
    m_odeSolver = nullptr;//OdeSolver 算积分 q（位置）, v（速度）
}

atg_scs::GenericRigidBodySystem::~GenericRigidBodySystem() {
    /* void */
}

//初始化，设置求解器以及中间值结构体
void atg_scs::GenericRigidBodySystem::initialize(
        SleSolver *sleSolver,
        OdeSolver *odeSolver)
{
    m_sleSolver = sleSolver;//设置SleSolver 求 λ的解算器
    m_odeSolver = odeSolver;//设置OdeSolver 算积分 q（位置）, v（速度）

    //初始化中间值矩阵的lambda为0
    m_iv.lambda.initialize(0, 0);
}

void atg_scs::GenericRigidBodySystem::process(double dt, int steps) {
    long long
        odeSolveTime = 0,
        constraintSolveTime = 0,
        forceEvalTime = 0,
        constraintEvalTime = 0;

    //填充系统状态，将刚体和约束的状态填充到系统状态中
    populateSystemState();
    //向列向量M、M_inv添加各个刚体的参数
    populateMassMatrices(&m_iv.M, &m_iv.M_inv);

    for (int i = 0; i < steps; ++i) {
        m_odeSolver->start(&m_state, dt / steps);

        while (true) {
            const bool done = m_odeSolver->step(&m_state);

            long long evalTime = 0, solveTime = 0;

            auto s0 = std::chrono::steady_clock::now();
            processForces();
            auto s1 = std::chrono::steady_clock::now();

            processConstraints(&evalTime, &solveTime);

            auto s2 = std::chrono::steady_clock::now();
            m_odeSolver->solve(&m_state);
            auto s3 = std::chrono::steady_clock::now();

            constraintSolveTime += solveTime;
            constraintEvalTime += evalTime;
            odeSolveTime +=
                std::chrono::duration_cast<std::chrono::microseconds>(s3 - s2).count();
            forceEvalTime +=
                std::chrono::duration_cast<std::chrono::microseconds>(s1 - s0).count();

            if (done) break;
        }

        m_odeSolver->end();
    }

    const int n = getRigidBodyCount();
    for (int i = 0; i < n; ++i) {
        m_rigidBodies[i]->v_x = m_state.v_x[i];
        m_rigidBodies[i]->v_y = m_state.v_y[i];

        m_rigidBodies[i]->p_x = m_state.p_x[i];
        m_rigidBodies[i]->p_y = m_state.p_y[i];

        m_rigidBodies[i]->v_theta = m_state.v_theta[i];
        m_rigidBodies[i]->theta = m_state.theta[i];
    }

    const int m = getConstraintCount();
    for (int i = 0, i_f = 0; i < m; ++i) {
        Constraint *constraint = m_constraints[i];

        for (int j = 0; j < constraint->getConstraintCount(); ++j, ++i_f) {
            for (int k = 0; k < constraint->m_bodyCount; ++k) {
                constraint->F_x[j][k] = m_state.r_x[i_f];
                constraint->F_y[j][k] = m_state.r_y[i_f];
                constraint->F_t[j][k] = m_state.r_t[i_f];
            }
        }
    }

    m_odeSolveMicroseconds[m_frameIndex] = odeSolveTime;
    m_constraintSolveMicroseconds[m_frameIndex] = constraintSolveTime;
    m_forceEvalMicroseconds[m_frameIndex] = forceEvalTime;
    m_constraintEvalMicroseconds[m_frameIndex] = constraintEvalTime;
    m_frameIndex = (m_frameIndex + 1) % ProfilingSamples;
}

void atg_scs::GenericRigidBodySystem::processConstraints(
        long long *evalTime,
        long long *solveTime)
{
    *evalTime = -1;
    *solveTime = -1;

    auto s0 = std::chrono::steady_clock::now();

    const int n = getRigidBodyCount();
    const int m_f = getFullConstraintCount();
    const int m = getConstraintCount();

    //设置q_dot向量
    m_iv.q_dot.resize(1, n * 3);// q_dot列向量:高3n宽1
    for (int i = 0; i < n; ++i) {
        m_iv.q_dot.set(0, i * 3 + 0, m_state.v_x[i]);
        m_iv.q_dot.set(0, i * 3 + 1, m_state.v_y[i]);
        m_iv.q_dot.set(0, i * 3 + 2, m_state.v_theta[i]);
    }

    m_iv.J_sparse.initialize(3 * n, m_f);
    m_iv.J_dot_sparse.initialize(3 * n, m_f);
    m_iv.ks.initialize(1, m_f);
    m_iv.kd.initialize(1, m_f);
    m_iv.C.initialize(1, m_f);

    Constraint::Output constraintOutput;
    //序号j for 约束对象
    //序号j_f for 约束方程
    for (int j = 0, j_f = 0; j < m; ++j) {
        //根据m_state中的信息(位置、速度、加速度)，计算约束的输出（C、J、J_dot、v_bias、limits、ks、kd）  
        m_constraints[j]->calculate(&constraintOutput, &m_state);

        //序号k for 当前约束对象内约束方程，j_f for 所有约束对象的约束方程累加
        const int n_f = m_constraints[j]->getConstraintCount();
        for (int k = 0; k < n_f; ++k, ++j_f) {
            //序号i for 当前约束对象内刚体数
            for (int i = 0; i < m_constraints[j]->m_bodyCount; ++i) {
                const int index = m_constraints[j]->m_bodies[i]->index;

                if (index == -1) continue;

                m_iv.J_sparse.setBlock(j_f, i, index);
                m_iv.J_dot_sparse.setBlock(j_f, i, index);
            }

            //序号i for 当前约束对象j内的刚体数*3 
            for (int i = 0; i < m_constraints[j]->m_bodyCount * 3; ++i) {
                const int index = m_constraints[j]->m_bodies[i / 3]->index;

                if (index == -1) continue;

                m_iv.J_sparse.set(j_f, i / 3, i % 3,
                        constraintOutput.J[k][i]);                              //J

                m_iv.J_dot_sparse.set(j_f, i / 3, i % 3,
                        constraintOutput.J_dot[k][i]);                          //J_dot

                m_iv.ks.set(0, j_f, constraintOutput.ks[k]);    //ks
                m_iv.kd.set(0, j_f, constraintOutput.kd[k]);    //kd
                m_iv.C.set(0, j_f, constraintOutput.C[k]);      //C
            }
        }
    }

    m_iv.J_sparse.multiply(m_iv.q_dot, &m_iv.reg0); //J·q_dot=reg0=C_dot
    //循环结束后，reg0可作他用。（下方代码使用reg0保存J·M_inv·F_ext的值）
    for (int i = 0; i < m_f; ++i) {
        m_iv.kd.set(0, i, m_iv.kd.get(0, i) * m_iv.reg0.get(0, i)); //计算kd·C_dot=kd·(J·q_dot)  计算结果存在m_iv.kd中。 
        m_iv.ks.set(0, i, m_iv.ks.get(0, i) * m_iv.C.get(0, i));    //计算ks·C  计算结果存在m_iv.ks中
    }

    //分配F_ext
    m_iv.F_ext.initialize(1, 3 * n, 0.0);
    for (int i = 0; i < n; ++i) {
        m_iv.F_ext.set(0, i * 3 + 0, m_state.f_x[i]);
        m_iv.F_ext.set(0, i * 3 + 1, m_state.f_y[i]);
        m_iv.F_ext.set(0, i * 3 + 2, m_state.t[i]);
    }

    m_iv.F_ext.leftScale(m_iv.M_inv, &m_iv.reg2);       //计算 M_inv·F_ext=reg2
    m_iv.J_sparse.multiply(m_iv.reg2, &m_iv.reg0);          //计算J·(M_inv·F_ext)=reg0 ,reg2内数据已使用 临时寄存器可做他用

    m_iv.J_dot_sparse.multiply(m_iv.q_dot, &m_iv.reg2);     //计算J_dot·q_dot=reg2
    m_iv.reg2.negate(&m_iv.reg1);                           //计算reg1= -reg2 = -J_dot·q_dot

    m_iv.reg1.subtract(m_iv.reg0, &m_iv.reg2);              //计算reg2= reg1 - reg0 = -J_dot·q_dot - J·M_inv·F_ext
    m_iv.reg2.subtract(m_iv.ks, &m_iv.reg0);                //计算reg0= reg2 - ks   = -J_dot·q_dot - J·M_inv·F_ext - ks·C
    m_iv.reg0.subtract(m_iv.kd, &m_iv.right);               //计算right= reg0 - kd  = -J_dot·q_dot - J·M_inv·F_ext - ks·C - kd·C_dot

    auto s1 = std::chrono::steady_clock::now();

    //求解 λ 的值
    //传入上一帧的 λ 作为初始猜测（热启动）。
    const bool solvable =
        m_sleSolver->solve(
            m_iv.J_sparse,
            m_iv.M_inv,
            m_iv.right,
            &m_iv.lambda,
            &m_iv.lambda);
    assert(solvable);

    auto s2 = std::chrono::steady_clock::now();

    // Constraint force derivation
    //  R = J_T * lambda_scale                                      计算约束力 R = Jᵀ · λ
    //  => transpose(J) * transpose(transpose(lambda_scale)) = R             Jᵀ·λᵀᵀ=R
    //  => transpose(lambda_scale * J) = R                                   (λ·J)ᵀ=R       
    //  => transpose(J.leftScale(lambda_scale)) = R

    m_iv.J_sparse.leftScale(m_iv.lambda, &m_iv.sreg0);      //计算(λ·J)ᵀ=sreg0

    //更新m_state中的约束反力 r_x, r_y, r_t
    for (int i = 0; i < m_f; ++i) {
        for (int j = 0; j < 2; ++j) {
            m_state.r_x[i * 2 + j] = m_iv.sreg0.get(i, j, 0);
            m_state.r_y[i * 2 + j] = m_iv.sreg0.get(i, j, 1);
            m_state.r_t[i * 2 + j] = m_iv.sreg0.get(i, j, 2);
        }
    }
    
    //合并加速度（先叠加力，最后一次性除以质量）：
    //用外力初始化
    for (int i = 0; i < n; ++i) {
        m_state.a_x[i] = m_iv.F_ext.get(0, i * 3 + 0);
        m_state.a_y[i] = m_iv.F_ext.get(0, i * 3 + 1);
        m_state.a_theta[i] = m_iv.F_ext.get(0, i * 3 + 2);
    }
    //叠加约束力
    for (int i = 0, j_f = 0; i < m; ++i) {
        Constraint *constraint = m_constraints[i];

        const int n_f = constraint->getConstraintCount();
        for (int j = 0; j < n_f; ++j, ++j_f) {
            for (int k = 0; k < constraint->m_bodyCount; ++k) {
                const int body = constraint->m_bodies[k]->index;
                m_state.a_x[body] += m_state.r_x[j_f * 2 + k];
                m_state.a_y[body] += m_state.r_y[j_f * 2 + k];
                m_state.a_theta[body] += m_state.r_t[j_f * 2 + k];
            }
        }
    }

    //除以质量  F=ma  => a=F/m
    for (int i = 0; i < n; ++i) {
        const double invMass = m_iv.M_inv.get(0, i * 3 + 0);
        const double invInertia = m_iv.M_inv.get(0, i * 3 + 2);

        m_state.a_x[i] *= invMass;
        m_state.a_y[i] *= invMass;
        m_state.a_theta[i] *= invInertia;
    }

    auto s3 = std::chrono::steady_clock::now();

    //
    *evalTime =
        std::chrono::duration_cast<std::chrono::microseconds>(s1 - s0 + s3 - s2).count();
    //计算约束求解时间
    *solveTime =
        std::chrono::duration_cast<std::chrono::microseconds>(s2 - s1).count();
}
