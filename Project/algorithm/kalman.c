#include <stdio.h>

//1. 结构体类型定义
// https://blog.csdn.net/CSDN_X_W/article/details/90289021
typedef struct 
{
    float LastP;    //上次估算协方差 初始化值为0.02
    float Now_P;    //当前估算协方差 初始化值为0
    float out;  //卡尔曼滤波器输出 初始化值为0
    float Kg;   //卡尔曼增益 初始化值为0
    float Q;    //过程噪声协方差 初始化值为0.001
    float R;    //观测噪声协方差 初始化值为0.543
} KFP; //Kalman Filter parameter

//2. 以高度为例 定义卡尔曼结构体并初始化参数
KFP KFP_height = {0.02, 0, 0, 0, 0.001, 0.543};

/**
 *卡尔曼滤波器
 *@param KFP *kfp 卡尔曼结构体参数
 *   float input 需要滤波的参数的测量值（即传感器的采集值）
 *@return 滤波后的参数（最优值）
 */
float kalmanFilter(KFP *kfp, float input)
{
    //预测协方差方程：k时刻系统估算协方差 = k-1时刻的系统协方差 + 过程噪声协方差
    kfp->Now_P = kfp->LastP + kfp->Q;
    //卡尔曼增益方程：卡尔曼增益 = k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）
    kfp->Kg = kfp->Now_P / (kfp->Now_P + kfp->R);
    //更新最优值方程：k时刻状态变量的最优值 = 状态变量的预测值 + 卡尔曼增益 * （测量值 - 状态变量的预测值）
    kfp->out = kfp->out + kfp->Kg * (input -kfp->out);//因为这一次的预测值就是上一次的输出值
    //更新协方差方程: 本次的系统协方差付给 kfp->LastP 威下一次运算准备。
    kfp->LastP = (1 - kfp->Kg) * kfp->Now_P;
     
    return kfp->out;
}

/**
 *调用卡尔曼滤波器 实践
 */
int main()
{
    int height, i;
    float kalman_height = 0;
    float test_data[] = {1, 10, 3, 9, 2, 7, 5, 6, 3, 2, 2, 1, 0};

    for (i = 0; i < sizeof(test_data) / sizeof(float); i++) {
        kalman_height = kalmanFilter(&KFP_height,(float)test_data[i]);
        printf("%02d: %f\n", i + 1, kalman_height);
    }
    printf("LastP: %f, Now_P: %f, Kg: %f, Q: %f, R: %f\n", KFP_height.LastP, KFP_height.Now_P, KFP_height.Kg, KFP_height.Q, KFP_height.R);
    
    return 0;
}