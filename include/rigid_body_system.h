#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_RIGID_BODY_SYSTEM_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_RIGID_BODY_SYSTEM_H

#include "rigid_body.h"
#include "constraint.h"
#include "force_generator.h"
//矩阵
#include "matrix.h"
//稀疏矩阵
#include "sparse_matrix.h"
#include "system_state.h"

#include <vector>

namespace atg_scs {
    class RigidBodySystem {
        public:
            //记录600个时间步长的性能数据?
            static const int ProfilingSamples = 60 * 10;

        public:
            RigidBodySystem();
            virtual ~RigidBodySystem();

            virtual void reset();
            //steps还能大于1？一次执行多步吗？
            virtual void process(double dt, int steps = 1);

            void addRigidBody(RigidBody *body);
            void removeRigidBody(RigidBody *body);
            RigidBody *getRigidBody(int i);

            void addConstraint(Constraint *constraint);
            void removeConstraint(Constraint *constraint);

            void addForceGenerator(ForceGenerator *generator);
            void removeForceGenerator(ForceGenerator *generator);

            int getRigidBodyCount() const { return (int)m_rigidBodies.size(); }
            //约束容器m_constraints中约束实例的数量
            int getConstraintCount() const { return (int)m_constraints.size(); }
            int getForceGeneratorCount() const { return (int)m_forceGenerators.size(); }

            int getFullConstraintCount() const;

            float getOdeSolveMicroseconds() const;
            float getConstraintSolveMicroseconds() const;
            float getForceEvalMicroseconds() const;
            float getConstraintEvalMicroseconds() const;

            inline const SystemState *state() const { return &m_state; }

        protected:
            static float findAverage(long long *samples);

            void populateSystemState();
            void populateMassMatrices(Matrix *M, Matrix *M_inv);
            void processForces();

        protected:
            std::vector<RigidBody *> m_rigidBodies;
            std::vector<Constraint *> m_constraints;
            std::vector<ForceGenerator *> m_forceGenerators;

            SystemState m_state;

            long long *m_odeSolveMicroseconds;
            long long *m_constraintSolveMicroseconds;
            long long *m_forceEvalMicroseconds;
            long long *m_constraintEvalMicroseconds;
            long long m_frameIndex;
    };
} /* namespace atg_scs */

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_RIGID_BODY_SYSTEM_H */
