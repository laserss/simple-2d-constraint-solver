#ifndef ATG_SIMPLE_2D_CONSTRAINT_SOLVER_SPARSE_MATRIX_H
#define ATG_SIMPLE_2D_CONSTRAINT_SOLVER_SPARSE_MATRIX_H

#include "matrix.h"

#include "utilities.h"

#include <assert.h>
#include <stdint.h>
#include <string.h>

namespace atg_scs {

    //模板？作用是啥
    //稀疏矩阵类
    //只存每行的非零块，每行最多 T_Entries=2 个非零块，每个非零块有T_Stride=3 个（J对此刚体的偏导数）值。
    //如何判断每行储存的两个块是哪些列？通过m_blockData数组记录每行两个块的编号。（逻辑上每行有n个块，但n-2个块都是0，剩下的2个块的编号记录在m_blockData中）
    //每一行：一个约束对多个关系坐标求偏导数，因为一个约束只连接2个刚体，所以T_Entries=2；广义坐标最多3个（x/y/theta），所以T_Stride=3
    //每一行需要存的数据数量=T_Entries*T_Stride=6个值,m_data就是用来存这些值的
    //矩阵的总行数与约束数量有关，N个约束就有N行，m_height=约束数量
    template <int T_Stride = 3, int T_Entries = 2>
    class SparseMatrix {
        public:
            SparseMatrix() {
                //储存矩阵每一行的指针（最多m_capacityHeight个指针），每个指针指向2个块（每个块占用2*T_Stride(3)=6个位置）/6个值
                m_matrix = nullptr;
                //储存矩阵数据,通过m_matrix的指针访问
                m_data = nullptr;
                //块数据是什么？
                //m_blockData:一维数组，大小为行数*T_Entries(2)=m_height*2。记录每一行的2个块对应哪个刚体编号（由于可能存在多个刚体0至n-1,需要记录哪两个刚体是非零的/与约束关联的），0xFF = 此槽位为空
                m_blockData = nullptr;
                m_width = m_height = 0;
                //为什么只有高度限制？
                //高度与约束数量有关，约束越多，高度（行数）越多
                //宽度与矩阵中非0值数量有关，而非零值每行最多2（一个约束对应2个刚体）*3（广义坐标数）=6个值，所以宽度为6
                m_capacityHeight = 0;
            }

            ~SparseMatrix() {
                assert(m_matrix == nullptr);
                assert(m_data == nullptr);
                assert(m_blockData == nullptr);
            }

            //忽略宽度，因为实际存储的宽度固定为6。高度根据输入参数变化。
            void initialize(int width, int height) {
                resize(width, height);
                memset(m_blockData, 0xFFFFFF, sizeof(uint8_t) * T_Entries * m_height);
            }
            //忽略宽度～
            void resize(int width, int height) {
                if (width == m_width && height == m_height) return;
                else if (height > m_capacityHeight) {
                    destroy();

                    //此处的判断是否多余？是的，不需要判断，可以直接赋值：m_capacityHeight = height;
                    m_capacityHeight = (height > m_capacityHeight)
                        ? height
                        : m_capacityHeight;

                    m_data = new double[(size_t)T_Stride * T_Entries * m_capacityHeight];
                    m_matrix = new double *[m_capacityHeight];
                    //
                    m_blockData = new uint8_t[(size_t)m_capacityHeight * T_Entries];
                }

                m_height = height;
                m_width = width;
                //将m_data中的数据按行绑定到m_matrix的每个指针
                //每一行有T_Entries * T_Stride个元素
                for (int i = 0; i < height; ++i) {
                    m_matrix[i] = &m_data[i * T_Entries * T_Stride];
                }
            }

            void destroy() {
                if (m_matrix == nullptr) {
                    return;
                }

                delete[] m_matrix;
                delete[] m_data;
                delete[] m_blockData;

                m_matrix = nullptr;
                m_data = nullptr;
                m_blockData = nullptr;

                m_width = m_height = 0;
            }

            //将稀疏矩阵扩张为密集矩阵（空白位置在matrix->initialize时被初始化为0）
            void expand(Matrix *matrix) {
                matrix->initialize(m_width, m_height);

                for (int i = 0; i < m_height; ++i) {
                    for (int j = 0; j < T_Entries; ++j) {
                        const uint8_t block = m_blockData[i * T_Entries + j];   //获取稀疏矩阵第i行第j个块的刚体编号
                        if (block == 0xFF) continue;   //如果块为空，则跳过
                        else {   //如果块不为空，则将块中的数据复制到目标矩阵中
                            for (int k = 0; k < T_Stride; ++k) {
                                matrix->set(block * T_Stride + k, i, m_matrix[i][j * T_Stride + k]);
                            }
                        }
                    }
                }
            }

            //将稀疏矩阵扩张为密集矩阵（空白位置在matrix->initialize时被初始化为0）,同时进行转置
            void expandTransposed(Matrix *matrix) {
                matrix->initialize(m_height, m_width);

                for (int i = 0; i < m_height; ++i) {
                    for (int j = 0; j < T_Entries; ++j) {
                        const uint8_t block = m_blockData[i * T_Entries + j];
                        if (block == 0xFF) continue;
                        else {
                            for (int k = 0; k < T_Stride; ++k) {
                                matrix->set(i, block * T_Stride + k, m_matrix[i][j * T_Stride + k]);
                            }
                        }
                    }
                }
            }

            //设置m_blockData中第row行第entry个位置刚体的index
            inline void setBlock(int row, int entry, uint8_t index) {
                assert(row >= 0 && row < m_height);
                assert(entry >= 0 && entry < T_Entries);    //取值0、1
                assert(index < m_width);

                m_blockData[row * T_Entries + entry] = index;
            }

            //设置m_matrix中第row行（虚拟行，实际存储为一维数组连续排列）第entry个块（每个块占用T_Stride=3个位置）的第slice个位置的值
            inline void set(int row, int entry, int slice, double v) {
                assert(row >= 0 && row < m_height);
                assert(entry >= 0 && entry < T_Entries);
                assert(slice < T_Stride);

                m_matrix[row][entry * T_Stride + slice] = v;
            }

            //读取m_martix数据。row:行索引，entry:行内块索引，slice:块内位置索引
            inline double get(int row, int entry, int slice) {
                assert(row >= 0 && row < m_height);
                assert(entry >= 0 && entry < T_Entries);
                assert(slice < T_Stride);

                return m_matrix[row][entry * T_Stride + slice];
            }

            //设置第m_matrix中第row行第col个块的3个值为0，同时将m_blockData中第row行第col个位置的刚体编号设置为0xFF
            inline void setEmpty(int row, int col) {
                assert(row >= 0 && row < m_height);
                assert(col >= 0 && col < T_Entries);

                m_blockData[row * T_Entries + col] = 0xFF;
                for (int i = 0; i < T_Stride; ++i) {
                    m_matrix[row][col * T_Stride + i] = 0;
                }
            }

            //计算本矩阵与b_T矩阵的转置的乘积，结果存储到target矩阵。用于计算系数矩阵，乘一个矩阵的转置
            //下行中出现了两个const，第一个表示不修改b_T矩阵，第二个表示不修改本矩阵
            void multiplyTranspose(const SparseMatrix<T_Stride, T_Entries> &b_T, Matrix *target) const {
                assert(m_width == b_T.m_width);

                target->initialize(b_T.m_height, m_height);

                for (int i = 0; i < m_height; ++i) {
                    for (int j = 0; j < b_T.m_height; ++j) {
                        double dot = 0;
                        for (int k = 0; k < T_Entries; ++k) {
                            const uint8_t block0 = m_blockData[i * T_Entries + k];
                            if (block0 == 0xFF) continue;

                            for (int l = 0; l < T_Entries; ++l) {
                                const uint8_t block1 = b_T.m_blockData[j * T_Entries + l];
                                //为什么编号相同才计算？矩阵乘法什么时候要考虑刚体编号了？
                                //因为此函数用于计算JM^(-1)J^(T),M为对角矩阵，M^-1仍是对角矩阵，乘一个对角矩阵等于对各行进行缩放
                                //所以不影响J中元素的排列，只影响结果矩阵的值
                                //J*J^(T)，计算结果的i行等于J矩阵的i行点乘J矩阵i行的转置，而J矩阵i行最多只有6个非零值（一个约束关联两个刚体）
                                //所以添加一个刚体编号是否相等。更多细节参见obisidian笔记：系数矩阵JWJ(T)的计算
                                if (block0 == block1) {
                                    for (int m = 0; m < T_Stride; ++m) {
                                        dot +=
                                            m_matrix[i][k * T_Stride + m]
                                            * b_T.m_matrix[j][l * T_Stride + m];
                                    }
                                }
                            }
                        }

                        target->set(j, i, dot);
                    }
                }
            }
            //本矩阵转置乘b向量（列向量）
            void transposeMultiplyVector(Matrix &b, Matrix *target) const {
                const int b_w = b.getWidth();
                const int b_h = b.getHeight();

                assert(b_w == 1);   //b是列向量
                assert(m_height == b_h);//本矩阵的转置能够和列向量b相乘

                //target宽度为1，高度为m_width，列向量
                target->initialize(1, m_width);

                for (int i = 0; i < m_height; ++i) {
                    double v = 0.0;
                    for (int k = 0; k < T_Entries; ++k) {
                        const int offset = k * T_Stride;
                        const uint8_t block = m_blockData[i * T_Entries + k];
                        if (block == 0xFF) continue;

                        for (int l = 0; l < T_Stride; ++l) {
                            const int j = block * T_Stride + l;
                            target->add(0, j, m_matrix[i][offset + l] * b.get(0, i));
                        }
                    }
                }
            }

            //本稀疏矩阵乘正常矩阵b
            void multiply(Matrix &b, Matrix *target) const {
                const int b_w = b.getWidth();
                const int b_h = b.getHeight();

                assert(m_width == b_h);

                target->initialize(b.getWidth(), m_height);

                for (int i = 0; i < m_height; ++i) {
                    for (int j = 0; j < b_w; ++j) {
                        double v = 0.0;
                        for (int k = 0; k < T_Entries; ++k) {
                            const int offset = k * T_Stride;
                            const uint8_t block = m_blockData[i * T_Entries + k];
                            if (block == 0xFF) continue;

                            for (int l = 0; l < T_Stride; ++l) {
                                v += m_matrix[i][offset + l] * b.get(j, block * T_Stride + l);
                            }
                        }

                        target->set(j, i, v);
                    }
                }
            }

            //对角矩阵scale*本矩阵：按scale矩阵的系数缩放本矩阵的各行
            void rightScale(Matrix &scale, SparseMatrix<T_Stride> *target) {
                assert(scale.getWidth() == 1);
                assert(scale.getHeight() == m_width);

                target->initialize(m_width, m_height);

                for (int i = 0; i < m_height; ++i) {
                    for (int j = 0; j < T_Entries; ++j) {
                        const uint8_t index = m_blockData[i * T_Entries + j];
                        if (index == 0xFF) continue;       //如果块为空，则跳过

                        target->setBlock(i, j, index);

                        for (int k = 0; k < T_Stride; ++k) {
                            target->set(
                                i,
                                j,
                                k,
                                scale.get(0, index * T_Stride + k) * m_matrix[i][j * T_Stride + k]);
                        }
                    }
                }
            }

            //本矩阵*对角矩阵scale：按scale矩阵的系数缩放本矩阵的各列
            void leftScale(Matrix &scale, SparseMatrix<T_Stride> *target) {
                assert(scale.getWidth() == 1 || m_height == 0);
                assert(scale.getHeight() == m_height);

                target->initialize(m_width, m_height);

                for (int i = 0; i < m_height; ++i) {
                    for (int j = 0; j < T_Entries; ++j) {
                        const uint8_t index = m_blockData[i * T_Entries + j];
                        if (index == 0xFF) continue;

                        target->setBlock(i, j, index);

                        for (int k = 0; k < T_Stride; ++k) {
                            target->set(
                                i,
                                j,
                                k,
                                scale.get(0, i) * m_matrix[i][j * T_Stride + k]);
                        }
                    }
                }
            }

            scs_force_inline int getWidth() const { return m_width; }
            scs_force_inline int getHeight() const { return m_height; }

        protected:
            double **m_matrix;
            double *m_data;
            uint8_t *m_blockData;

            int m_width;
            int m_height;
            int m_capacityHeight;
    };
} /* namespace atg_scs */

#endif /* ATG_SIMPLE_2D_CONSTRAINT_SOLVER_SPARSE_MATRIX_H */
