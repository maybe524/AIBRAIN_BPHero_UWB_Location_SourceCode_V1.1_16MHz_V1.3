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
    double LastP;    // 涓婃浼扮畻鍗忔柟宸� 鍒濆鍖栧�间负0.02
    double Now_P;    // 褰撳墠浼扮畻鍗忔柟宸� 鍒濆鍖栧�间负0
    double out;  // 鍗″皵鏇兼护娉㈠櫒杈撳嚭 鍒濆鍖栧�间负0
    double Kg;   // 鍗″皵鏇煎鐩� 鍒濆鍖栧�间负0
    double Q;    // 杩囩▼鍣０鍗忔柟宸� 鍒濆鍖栧�间负0.001
    double R;    // 瑙傛祴鍣０鍗忔柟宸� 鍒濆鍖栧�间负0.543
} KFP; // Kalman Filter parameter

double kalmanFilter(KFP *kfp, double input);

#endif

