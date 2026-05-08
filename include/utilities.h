#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_UTILITIES_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_UTILITIES_H


//macos下使用inline，windows下使用__forceinline
//inline是内联函数，__forceinline是强制内联函数
//macOS（Apple clang）：使用 inline，向编译器建议内联，编译器可自行决定
//Windows（MSVC）：使用 __forceinline，强制编译器内联，不允许编译器拒绝
//Windows 下用强制内联是因为 MSVC 的优化策略有时会忽略 inline 建议，用 __forceinline 确保行为一致。

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
