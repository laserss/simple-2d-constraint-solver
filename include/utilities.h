#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_UTILITIES_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_UTILITIES_H


//macos下使用inline，windows下使用__forceinline
//inline是内联函数，__forceinline是强制内联函数
//Windows为什么要强制内联函数？
//因为Windows下，inline函数可能会被编译器优化，导致函数调用不准确
//所以需要强制内联函数，确保函数调用准确
#if defined(__APPLE__)
#define scs_force_inline inline
#else
#define scs_force_inline __forceinline
#endif

namespace atg_scs {
    //释放double数组，并将data设置为nullptr
    void freeArray(double *&data);
    //释放int数组，并将data设置为nullptr
    void freeArray(int *&data);
} /* atg_scs */

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_UTILITIES_H */
