#ifndef _KALMAN_H
#define _KALMAN_H


#define   LENGTH      1*1
#define   ORDER       1
#define   N           100
#define   SEED        1567

//================================================//
//==               ����ֵ�ṹ��                 ==//
//================================================//
typedef struct  _tOptimal
{
  float XNowOpt[LENGTH];
  float XPreOpt[LENGTH];
} tOptimal;

extern void   KalMan_PramInit(void);
extern float KalMan_Update(double *Z);


// Aibrain code
// 1. 结构体类型定义
typedef struct {
    float LastP;    // 上次估算协方差 初始化值为0.02
    float Now_P;    // 当前估算协方差 初始化值为0
    float out;  // 卡尔曼滤波器输出 初始化值为0
    float Kg;   // 卡尔曼增益 初始化值为0
    float Q;    // 过程噪声协方差 初始化值为0.001
    float R;    // 观测噪声协方差 初始化值为0.543
} KFP; // Kalman Filter parameter

float kalmanFilter(KFP *kfp, float input);

#endif

