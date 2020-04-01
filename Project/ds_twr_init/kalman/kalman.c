/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"	
#include "kalman.h"	
#include "matrix.h"				 				 		

/*==============================================================================
1.Ԥ����
   X(k|k-1) = F(k,k-1)*X(k-1|k-1)        //������Ϊ0


2.����Ԥ����Э�������
   P(k|k-1) = F(k,k-1)*P(k-1|k-1)*F(k,k-1)'+Q(k)
   Q(k) = U(k)��U(k)' 


3.���㿨�����������
   Kg(k) = P(k|k-1)*H' / (H*P(k|k-1)*H' + R(k))
   R(k) = N(k)��N(k)' 


4.���¹���
   X(k|k) = X(k|k-1)+Kg(k)*(Z(k)-H*X(k|k-1))


5.������º����Э�������
   P(k|k) =��I-Kg(k)*H��*P(k|k-1)


6. ��������ֵ


F(k,k-1):     ״̬ת�ƾ���
X(k|k-1):     ����k-1ʱ�̵�����ֵ����kʱ�̵�ֵ
X(k-1|k-1):   k-1ʱ�̵�����ֵ
P(k|k-1):     X(k|k-1)��Ӧ��covariance
P(k-1|k-1):   X(k-1|k-1)��Ӧ��covariance
Q(k):         ϵͳ���̵�covariance
R(k):         �������̵�Э����
H(k):         �۲����ת�ƾ���
Z(k):         kʱ�̵Ĳ���ֵ


����˼·: ���ȸ�����һ��(����ǵ�һ�������Ԥ��ֵ����)�����ݼ�������εĹ���ֵ,
          ͬ��,������һ�ε����ݼ�������ι���ֵ��Э����;  ����,�ɱ��ι���ֵ��Э
          ������������������;  ���,���ݹ���ֵ�Ͳ���ֵ���㵱ǰ����ֵ����Э����
==============================================================================*/



//================================================//
//==             ����ֵ����ṹ��               ==//
//================================================//
typedef struct  _tCovariance
{
  float PNowOpt[LENGTH];
  float PPreOpt[LENGTH];
}tCovariance;







static tOptimal      tOpt;
static tCovariance   tCov;
//float         Z[LENGTH]  = {4000};           //  ����ֵ(ÿ�β�����������Ҫ���������)
static float         I[LENGTH]  = {1};              //  ��λ����
static float         X[LENGTH]  = {9.8};              //  ��ǰ״̬��Ԥ��ֵ
static float         P[LENGTH]  = {0};              //  ��ǰ״̬��Ԥ��ֵ��Э����
static float         K[LENGTH]  = {0};              //  ����������
static float         Temp3[LENGTH] = {0};           //  ��������
//============================================================================//
//==                    �������˲���Ҫ���õı���                            ==//
//============================================================================//
static float         F[LENGTH]  = {1};              //  ״̬ת�ƾ���   �������ŵ�ǰ��״̬����һ��״̬�Ĺ�ϵ�����������һ���ģ���״̬ת�ƾ����Ϊ��λ����
static float         Q[LENGTH]  = {0.0001f};//0.0001f              //  ϵͳ���̵�Э����	Э����Ķ��壺��ʵֵ������ֵ֮���ƽ��������ֵ
static float         R[LENGTH]  = {2};              //  �������̵�Э����	Э����Ķ��壺��ʵֵ������ֵ֮���ƽ��������ֵ   
//�������Ҫ�˲�����������ڹ۲������Ǿ͵�СR������Q����֮������R����СQ����������ֵ��ȡ����ϵͳ��
//���R��QС������˵��״̬����ֵ�Ȳ���ֵҪ�ɿ�����ʱ�����ó��Ľ�����Ǹ��ӽ�����ֵ��
//���RСQ����ʱ����������Ľ���ͻ���ӽ�����ֵ��
static float         H[LENGTH]  = {1};              //  �۲����ת�ƾ���	����ֵ��״̬Ԥ��ֵ֮��ĵ�λ�����ϵ������Ԥ��ֵ��λ����ɲ���ֵ��λ
static float         Temp1[LENGTH] = {1};           //  ��������, ͬʱ����tOpt.XPreOpt[]�ĳ�ʼ��ֵ
static float         Temp2[LENGTH] = {10000};       //  ��������, ͬʱ����tCov.PPreOpt[]�ĳ�ʼ��ֵ


void KalMan_PramInit(void)
{
  unsigned char   i;
  
  for (i=0; i<LENGTH; i++)
  {
    tOpt.XPreOpt[i] = Temp1[i];           //��ֵ��ʼ��
  }
  for (i=0; i<LENGTH; i++)
  {
    tCov.PPreOpt[i] = Temp2[i];           //��ֵ��ʼ��
  }
}


//============================================================================//
//==                          �������˲�                                    ==//
//============================================================================//
//==��ڲ���: ��ǰʱ�̵Ĳ���ֵ                                                            ==//
//==���ڲ���: ��ǰʱ�̵�����ֵ                                                            ==//
//==����ֵ:   ��ǰʱ�̵�����ֵ                                                            ==//
//============================================================================//
float KalMan_Update(double *Z)
{
	u8 i;  
	MatrixMul(F, tOpt.XPreOpt, X, ORDER, ORDER, ORDER);       //  ����ϵͳ����һ״̬��Ԥ������״̬; X(k|k-1) = F(k,k-1)*X(k-1|k-1)

	MatrixCal(F, tCov.PPreOpt, Temp1, ORDER);
	MatrixAdd(Temp1, Q, P, ORDER, ORDER);                     //  Ԥ�����ݵ�Э�������; P(k|k-1) = F(k,k-1)*P(k-1|k-1)*F(k,k-1)'+Q

	MatrixCal(H, P, Temp1, ORDER);
	MatrixAdd(Temp1, R, Temp1, ORDER, ORDER);
	Gauss_Jordan(Temp1, ORDER);
	MatrixTrans(H, Temp2, ORDER, ORDER);
	MatrixMul(P, Temp2, Temp3, ORDER, ORDER, ORDER);
	MatrixMul(Temp1, Temp3, K, ORDER, ORDER, ORDER);          //  ���㿨��������; Kg(k) = P(k|k-1)*H' / (H*P(k|k-1)*H' + R)

	MatrixMul(H, X, Temp1, ORDER, ORDER, ORDER);
	MatrixMinus(Z, Temp1, Temp1, ORDER, ORDER);
	MatrixMul(K, Temp1, Temp2, ORDER, ORDER, ORDER);
	MatrixAdd(X, Temp2, tOpt.XNowOpt, ORDER, ORDER);          //  ���ݹ���ֵ�Ͳ���ֵ���㵱ǰ����ֵ; X(k|k) = X(k|k-1)+Kg(k)*(Z(k)-H*X(k|k-1))

	MatrixMul(K, H, Temp1, ORDER, ORDER, ORDER);
	MatrixMinus((double *)I, Temp1, Temp1, ORDER, ORDER);
	MatrixMul(Temp1, P, tCov.PNowOpt, ORDER, ORDER, ORDER);   //  ������º����Э�������; P(k|k) =��I-Kg(k)*H��*P(k|k-1)

	for (i=0; i<LENGTH; i++)
	{
	  tOpt.XPreOpt[i] = tOpt.XNowOpt[i];
	  tCov.PPreOpt[i] = tCov.PNowOpt[i];
	}
	
	return tOpt.XNowOpt[0];
}
