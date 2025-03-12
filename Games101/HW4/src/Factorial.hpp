#ifndef FACTORIAL_H_
#define FACTORIAL_H_

/**
 * @brief 计算阶乘的函数
 * @param n 计算阶乘的数据
 * @return decltype(n) 计算的结果
 * @note 递归的计算方式
 * @note 阶乘的计算会发生爆炸 超过int 的范围 需要别的方式计算，需要一个稳定的 不会爆炸的算法
 */

class factorial
{
public:
    int operator()(int n)
    {
        if (n == 0)
        {
            return 1;
        }

        return n * factorial()(n - 1);
    }
};

#endif //! FACTORIAL_H_
