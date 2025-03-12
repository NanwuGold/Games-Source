#ifndef COMBINATION_H_
#define COMBINATION_H_

/**
 * @brief 计算二项式
 *
 * @param n
 * @param k
 * @return float 计算的结果
 * @note 存在爆炸风险
 */

class Combination
{
public:
    float operator()(int n, int k)
    {
        if (k > n)
        {
            return 0.0f;
        }

        if (0 == k || n == k)
        {
            return 1.0f;
        }

        k = (k < n - k) ? k : n - k; // 优化计算次数

        auto result = 1.0f;
        for (auto i = k; i >= 1; i--)
        {
            result *= static_cast<float>(n - i + 1) / i;
        }
        return result;
    }
};

#endif //! COMBINATION_H_