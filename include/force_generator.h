#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_FORCE_GENERATOR_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_FORCE_GENERATOR_H

#include "system_state.h"

namespace atg_scs {
    class ForceGenerator {
        public:
            ForceGenerator();

            // 析构函数为什么设置为虚函数？
            /*
                C++ 中将基类析构函数设置为虚函数（virtual），核心目的是为了在通过基类指针或引
                用删除派生类对象时，确保能够触发动态绑定，依次调用派生类和基类的析构函数，从而
                正确地释放所有内存和资源，防止内存泄漏。如果不设置，将只能调用基类析构函数，导
                致派生类特有资源未被释放。
            */
            virtual ~ForceGenerator();

            //虚函数=0 表示这是一个纯虚函数，必须在子类中实现
            virtual void apply(SystemState *system) = 0;
            /*
                C++中的普通虚函数（virtual）在子类中不强制必须实现，如果子类不重写（override），
                则默认调用基类的实现。但若是纯虚函数（virtual void func() = 0;），
                则子类必须实现，否则子类将因拥有纯虚函数而成为抽象类，无法实例化。
            */
            int m_index;
    };
} /* namespace atg_scs */

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_FORCE_GENERATOR_H */
