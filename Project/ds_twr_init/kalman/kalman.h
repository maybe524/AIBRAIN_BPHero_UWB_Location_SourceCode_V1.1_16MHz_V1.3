#ifndef _KALMAN_H
#define _KALMAN_H


#define   LENGTH      1*1
#define   ORDER       1
#define   N           100
#define   SEED        1567

//================================================//
//==               最优值结构体                 ==//
//================================================//
typedef struct  _tOptimal
{
  float XNowOpt[LENGTH];
  float XPreOpt[LENGTH];
} tOptimal;

extern void   KalMan_PramInit(void);
extern float KalMan_Update(double *Z);


// Aibrain code
// 1. 缁撴瀯浣撶被鍨嬪畾涔�
typedef struct {
    float LastP;    // 涓婃浼扮畻鍗忔柟宸� 鍒濆鍖栧�间负0.02
    float Now_P;    // 褰撳墠浼扮畻鍗忔柟宸� 鍒濆鍖栧�间负0
    float out;  // 鍗″皵鏇兼护娉㈠櫒杈撳嚭 鍒濆鍖栧�间负0
    float Kg;   // 鍗″皵鏇煎鐩� 鍒濆鍖栧�间负0
    float Q;    // 杩囩▼鍣０鍗忔柟宸� 鍒濆鍖栧�间负0.001
    float R;    // 瑙傛祴鍣０鍗忔柟宸� 鍒濆鍖栧�间负0.543
} KFP; // Kalman Filter parameter

float kalmanFilter(KFP *kfp, float input);

#endif

