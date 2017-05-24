#pragma once
#include"particle.h"
#include "MatchSolver.h"
#include "thread_variables.h"
//#include "MatchPSO.h"
using namespace std;


#define  e_PSO 0.001
#define CNT_PSO 30000
#define UP_PSO 1 //速度控制
#define DOWN_PSO -1
#define MAX_PSO 999999
#define WMAX_PSO 1.1
#define WMIN_PSO 0.9
#define STEP_PSO 3.0                    // 随机摄动步长
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
	CostFactorPyramid* cost_factor_pyramid;
	double init_x[6];
	double up_PSO[6];
	double down_PSO[6];
public:
	PSO(Mat &cam_img_input, double* init_pos, double * init_quat) 
	:MatchSolver(cam_img_input)
	{	
		particle = NULL;
		cost_factor_pyramid = new CostFactorPyramid(cam_img_input, 0);//暂时不用金字塔加速
		init_x[0] = init_pos[0];
		init_x[1] = init_pos[1];
		init_x[2] = init_pos[2];
		init_x[3] = init_quat[0]*rho_quat;
		init_x[4] = init_quat[1]* rho_quat;
		init_x[5] = init_quat[2]* rho_quat;

		up_PSO[0] = init_pos[0] + 20;
		up_PSO[1] = init_pos[1] + 20;
		up_PSO[2] = init_pos[2] + 20;
		up_PSO[3] = std::min(init_quat[1] + 50,1.0*rho_quat);
		up_PSO[4] = std::min(init_quat[2] + 50, 1.0*rho_quat);
		up_PSO[5] = std::min(init_quat[3] + 50, 1.0*rho_quat);

		down_PSO[0] = init_pos[0] - 20;
		down_PSO[1] = init_pos[1] - 20;
		down_PSO[2] = init_pos[2] - 20;
		down_PSO[3] = std::max(init_quat[1] - 50, -1.0*rho_quat);
		down_PSO[4] = std::max(init_quat[2] - 50, -1.0*rho_quat);
		down_PSO[5] = std::max(init_quat[3] - 50, -1.0*rho_quat);
	};
    PSO(Mat &cam_img_input, double* init_pos, double* init_quat, int pn, int d )
		//pn 粒子数量，d维度
		:MatchSolver(cam_img_input)
    {
		init_x[0] = init_pos[0];
		init_x[1] = init_pos[1];
		init_x[2] = init_pos[2];
		init_x[3] = init_quat[0]* rho_quat;
		init_x[4] = init_quat[1]* rho_quat;
		init_x[5] = init_quat[2]* rho_quat;

		up_PSO[0] = init_pos[0] + 20;
		up_PSO[1] = init_pos[1] + 20;
		up_PSO[2] = init_pos[2] + 20;
		up_PSO[3] = std::min(init_quat[1] + 50, 1.0*rho_quat);
		up_PSO[4] = std::min(init_quat[2] + 50, 1.0*rho_quat);
		up_PSO[5] = std::min(init_quat[3] + 50, 1.0*rho_quat);

		down_PSO[0] = init_pos[0] - 20;
		down_PSO[1] = init_pos[1] - 20;
		down_PSO[2] = init_pos[2] - 20;
		down_PSO[3] = std::max(init_quat[1] - 50, -1.0*rho_quat);
		down_PSO[4] = std::max(init_quat[2] - 50, -1.0*rho_quat);
		down_PSO[5] = std::max(init_quat[3] - 50, -1.0*rho_quat);

		cost_factor_pyramid = new CostFactorPyramid(cam_img_input, 0);
        pNum = pn;
        dim = d;//维度
		particle = NULL;
       // particle = new Particle[pn];         // 创建粒子数组实例对象

      //  for( int i=0; i<pn; ++i )
     //   {
    //        particle[i].setDim(d);           // 设置粒子的维数
     //   }
    }

    double function( double *x );               //  待求解函数
    void initial();
    void createNew(int n);                         // 生成新的粒子速度和位置
    bool isExited(int n);
	void doPSO(double * p_best, double &score_best);                          //
    void print( double *x );
    void setPNum(int n){ pNum = n;};           // 设置粒子数量
    void setDim( int d){ dim = d;};           // 设置维数

};
