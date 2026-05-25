#include "../include/matrix.h"

#include <algorithm>
#include <assert.h>

atg_scs::Matrix::Matrix() {
    m_matrix = nullptr;
    m_data = nullptr;
    m_width = m_height = 0;
    m_capacityWidth = m_capacityHeight = 0;
}

//构造函数内的执行：resize->initialize->Matrix
atg_scs::Matrix::Matrix(int width, int height, double value) {
    m_matrix = nullptr;
    m_data = nullptr;
    m_width = m_height = 0;
    m_capacityWidth = m_capacityHeight = 0;

    initialize(width, height, value);
}

atg_scs::Matrix::~Matrix() {
    assert(m_matrix == nullptr);
}

void atg_scs::Matrix::initialize(int width, int height, double value) {
    resize(width, height);

    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            m_matrix[i][j] = value;
        }
    }
}

void atg_scs::Matrix::initialize(int width, int height) {
    resize(width, height);
    memset(m_data, 0, sizeof(double) * width * height);
}

//修改矩阵的宽度高度，不处理矩阵内的数据
void atg_scs::Matrix::resize(int width, int height) {
    //如果宽度高度不变，则直接返回
    if (width == m_width && height == m_height) return;
    //如果宽度或高度大于最大宽度高度，则销毁矩阵并按照需要的大小重新申请空间
    else if (width > m_capacityWidth || height > m_capacityHeight) {
        destroy();

        m_capacityWidth = (width > m_capacityWidth)
            ? width
            : m_capacityWidth;

        m_capacityHeight = (height > m_capacityHeight)
            ? height
            : m_capacityHeight;

        //申请空间
        //size_t表示unsigned long 无符号长整型，用于将m_capacityWidth * m_capacityHeight转换为无符号长整型，防止整数溢出
        m_data = new double[(size_t)m_capacityWidth * m_capacityHeight];
        m_matrix = new double *[m_capacityHeight];
    }

    m_height = height;
    m_width = width;

    //将m_data中的数据按每width个元素计一行，按顺序绑定到m_matrix的各个指针
    for (int i = 0; i < height; ++i) {
        m_matrix[i] = &m_data[i * width];
    }
}

void atg_scs::Matrix::destroy() {
    if (m_matrix == nullptr) {
        return;
    }

    delete[] m_matrix;
    delete[] m_data;

    m_matrix = nullptr;
    m_data = nullptr;

    m_width = m_height = 0;
    m_capacityWidth = m_capacityHeight = 0;
}

//复制double数组data的内容，粘贴到m_data中
void atg_scs::Matrix::set(const double *data) {
    memcpy(m_data, data, sizeof(double) * m_width * m_height);
}

//复制矩阵 reference 的内容
void atg_scs::Matrix::set(Matrix *reference) {
    resize(reference->m_width, reference->m_height);

    for (int i = 0; i < reference->m_height; ++i) {
        for (int j = 0; j < reference->m_width; ++j) {
            m_matrix[i][j] = reference->m_matrix[i][j];
        }
    }
}

//下列函数参数列表中的&和*的含义：
//& → 这是输入，放心用，不会是空的
//* → 这是输出，调用时记得取地址，函数会往里写东西

//矩阵乘法,计算本矩阵乘矩阵b，结果存储到target矩阵中
void atg_scs::Matrix::multiply(Matrix &b, Matrix *target) {
    assert(m_width == b.m_height);

    target->resize(b.m_width, m_height);

    for (int i = 0; i < m_height; ++i) {
        for (int j = 0; j < b.m_width; ++j) {
            double v = 0.0;
            for (int ii = 0; ii < m_width; ++ii) {
                v += m_matrix[i][ii] * b.m_matrix[ii][j];
            }

            target->m_matrix[i][j] = v;
        }
    }
}

//矩阵逐元素乘法,计算本矩阵逐元素乘矩阵b，结果存储到target矩阵中
void atg_scs::Matrix::componentMultiply(Matrix &b, Matrix *target) {
    assert(m_height == b.m_height);
    assert(m_width == b.m_width);

    target->resize(m_width, m_height);

    for (int i = 0; i < m_height; ++i) {
        for (int j = 0; j < m_width; ++j) {
            target->set(j, i, get(j, i) * b.get(j, i));
        }
    }
}

//矩阵转置乘法,计算本矩阵的转置乘矩阵b，结果存储到target矩阵中
void atg_scs::Matrix::transposeMultiply(Matrix &b, Matrix *target) {
    assert(m_height == b.m_height);

    target->resize(b.m_width, m_width);

    for (int i = 0; i < m_width; ++i) {
        for (int j = 0; j < b.m_width; ++j) {
            double v = 0.0;
            for (int ii = 0; ii < m_height; ++ii) {
                v += m_matrix[ii][i] * b.m_matrix[ii][j];
            }

            target->m_matrix[i][j] = v;
        }
    }
}

//scale矩阵乘本矩阵，结果存储到target矩阵中
//scale作为列向量无法与本矩阵相乘，本函数是优化后的方案。
//原计算过程是scale作为对角矩阵(对角线\为scale)左乘本矩阵
//效果：本矩阵每一行缩放scale.m_matrix[i][0]
void atg_scs::Matrix::leftScale(Matrix &scale, Matrix *target) {
    assert(scale.m_width == 1);//scale矩阵必须为列向量
    assert(scale.m_height == m_height);//scale矩阵必须与本矩阵高度相同

    target->resize(m_width, m_height);

    for (int i = 0; i < m_height; ++i) {
        for (int j = 0; j < m_width; ++j) {
            target->m_matrix[i][j] = scale.m_matrix[i][0] * m_matrix[i][j];
        }
    }
}

//本矩阵乘scale矩阵，结果存储到target矩阵中
//scale作为行向量无法与本矩阵相乘，本函数是优化后的方案。
//原计算过程是scale作为对角矩阵(对角线\为scale)右乘本矩阵
//效果：本矩阵每一列缩放scale.m_matrix[j][0]
void atg_scs::Matrix::rightScale(Matrix &scale, Matrix *target) {
    assert(scale.m_width == 1);
    assert(scale.m_height == m_width);

    target->resize(m_width, m_height);

    for (int i = 0; i < m_height; ++i) {
        for (int j = 0; j < m_width; ++j) {
            target->m_matrix[i][j] = scale.m_matrix[j][0] * m_matrix[i][j];//每一列都乘了相同的数值：scale.m_matrix[j][0]
        }
    }
}

//矩阵缩放，将本矩阵的所有元素乘以s，结果存储到target矩阵中
void atg_scs::Matrix::scale(double s, Matrix *target) {
    target->resize(m_width, m_height);

    for (int i = 0; i < m_height; ++i) {
        for (int j = 0; j < m_width; ++j) {
            target->m_matrix[i][j] = s * m_matrix[i][j];
        }
    }
}

//矩阵减法,计算本矩阵减矩阵b（相同规格的矩阵），结果存储到target矩阵中
void atg_scs::Matrix::subtract(Matrix &b, Matrix *target) {
    assert(b.m_width == m_width);
    assert(b.m_height == m_height);

    target->resize(m_width, m_height);

    for (int i = 0; i < m_height; ++i) {
        for (int j = 0; j < m_width; ++j) {
            target->m_matrix[i][j] = m_matrix[i][j] - b.m_matrix[i][j];
        }
    }
}

//矩阵加法,计算本矩阵加矩阵b(相同规格的矩阵)，结果存储到target矩阵中
void atg_scs::Matrix::add(Matrix &b, Matrix *target) {
    assert(b.m_width == m_width);
    assert(b.m_height == m_height);

    target->resize(m_width, m_height);

    for (int i = 0; i < m_height; ++i) {
        for (int j = 0; j < m_width; ++j) {
            target->m_matrix[i][j] = m_matrix[i][j] + b.m_matrix[i][j];
        }
    }
}

//矩阵取反,计算本矩阵取反，结果存储到target矩阵中
void atg_scs::Matrix::negate(Matrix *target) {
    target->resize(m_width, m_height);

    for (int i = 0; i < m_height; ++i) {
        for (int j = 0; j < m_width; ++j) {
            target->m_matrix[i][j] = -m_matrix[i][j];
        }
    }
}

//矩阵相等判断,判断本矩阵与矩阵b是否相等，误差为err
bool atg_scs::Matrix::equals(Matrix &b, double err) {
    if (getWidth() != b.getWidth()) return false;
    if (getHeight() != b.getHeight()) return false;

    for (int i = 0; i < getHeight(); ++i) {
        for (int j = 0; j < getWidth(); ++j) {
            if (std::abs(get(j, i) - b.get(j, i)) > err) {
                return false;
            }
        }
    }

    return true;
}

//向量模平方,计算本矩阵的模平方
//用途1：计算列向量的模的平方（欧几里得范数的平方）
//用途2：在共轭梯度求解器中用于计算残差向量的大小，判断迭代是否收敛
double atg_scs::Matrix::vectorMagnitudeSquared() const {
    assert(m_width == 1);

    double mag = 0;
    for (int i = 0; i < m_height; ++i) {
        mag += m_matrix[0][i] * m_matrix[0][i];
        //因为是列向量，每一行只占用一个单元。所以在连续的内存单元中排列，看起来和行向量一样。
        //使用行向量的读取方式访问这个二维数组，就不用多次通过指针定位，提高了访问效率。
    }

    return mag;
}

//返回列向量点乘结果
double atg_scs::Matrix::dot(Matrix &b) const {
    assert(m_width == 1);
    assert(b.m_width == 1);
    assert(b.m_height == m_height);

    double result = 0;
    for (int i = 0; i < m_height; ++i) {
        result += m_matrix[0][i] * b.m_matrix[0][i];
    }

    return result;
}

//本矩阵累加s倍的矩阵b(相同规格的矩阵)，结果存储到本矩阵中
void atg_scs::Matrix::madd(Matrix &b, double s) {
    assert(m_width == b.m_width);
    assert(m_height == b.m_height);

    for (int i = 0; i < m_height; ++i) {
        for (int j = 0; j < m_width; ++j) {
            m_matrix[i][j] += b.m_matrix[i][j] * s;
        }
    }
}

//s倍的本矩阵+矩阵b，结果存储到本矩阵中
void atg_scs::Matrix::pmadd(Matrix &b, double s) {
    assert(m_width == b.m_width);
    assert(m_height == b.m_height);

    for (int i = 0; i < m_height; ++i) {
        for (int j = 0; j < m_width; ++j) {
            m_matrix[i][j] = s * m_matrix[i][j] + b.m_matrix[i][j];
        }
    }
}

//将本矩阵的转置储存到target矩阵中
void atg_scs::Matrix::transpose(Matrix *target) {
    target->resize(m_height, m_width);

    for (int i = 0; i < m_width; ++i) {
        for (int j = 0; j < m_height; ++j) {
            target->m_matrix[i][j] = m_matrix[j][i];
        }
    }
}
