#pragma once
#include"particle.h"
#include "MatchSolver.h"
#include "thread_variables.h"
//#include "MatchPSO.h"
using namespace std;


#define  e_PSO 0.001
#define CNT_PSO 30000
#define UP_PSO 10
#define DOWN_PSO -10
#define MAX_PSO 999999
#define WMAX_PSO 1.1
#define WMIN_PSO 0.9
#define STEP_PSO 1.0                    // 随机摄动步长
#define PI 3.1415926

class PSO: public MatchSolver
{
protected:
    Particle *particle;             // 粒子
    int pNum;                       // 粒子数量
    int dim;                        // 维数
    double **p;                     // 局部最优 某一粒子整个过程的最优位置
    int bestIndex;                  // 全局最优对应的粒子的索引值
    double fg;                      // 全局最优
    double lastFG;                  // 前一迭代过程的全局最优
    double c1;						//该例子历史最佳的学习参数
    double c2;						//所有点历史最佳的学习参数
    double w;						//惯性参数，影响全局搜索性能与局部收敛速度
    double *posNew;
    int times2;               // 循环计数器
    int mode;                 // 选择方程
public:
	PSO(Mat &cam_img_input, double* init_pos, double * init_quat) 
	:MatchSolver(cam_img_input)
	{	
		particle = NULL;
	};
    PSO(Mat &cam_img_input, double* init_pos, double* init_quat, int pn, int d )
		//pn 粒子数量，d维度
		:MatchSolver(cam_img_input)
    {
		
        pNum = pn;
        dim = d;//维度

        particle = new Particle[pn];         // 创建粒子数组实例对象

        for( int i=0; i<pn; ++i )
        {
            particle[i].setDim(d);           // 设置粒子的维数
        }
    }

    double function( double *x );               //  待求解函数
    void initial();
    void createNew(int n);                         // 生成新的粒子速度和位置
    bool isExited(int n);
	void doPSO(double * p_best, double &output_best);                           //
    void print( double *x );
    void setPNum(int n){ pNum = n;};           // 设置粒子数量
    void setDim( int d){ dim = d;};           // 设置维数

};
