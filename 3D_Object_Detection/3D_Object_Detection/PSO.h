#pragma once
#include"particle.h"
#include "MatchSolver.h"
#include "thread_variables.h"
//#include "MatchPSO.h"
using namespace std;


#define  e_PSO 0.001
#define CNT_PSO 30000
#define UP_PSO 1 //�ٶȿ���
#define DOWN_PSO -1
#define MAX_PSO 999999
#define WMAX_PSO 1.1
#define WMIN_PSO 0.9
#define STEP_PSO 3.0                    // ����㶯����
#define PI 3.1415926

class PSO: public MatchSolver
{
protected:
    Particle *particle;             // ����
    int pNum;                       // ��������
    int dim;                        // ά��
    double **p;                     // �ֲ����� ĳһ�����������̵�����λ��
    int bestIndex;                  // ȫ�����Ŷ�Ӧ�����ӵ�����ֵ
    double fg;                      // ȫ������
    double lastFG;                  // ǰһ�������̵�ȫ������
    double c1;						//��������ʷ��ѵ�ѧϰ����
    double c2;						//���е���ʷ��ѵ�ѧϰ����
    double w;						//���Բ�����Ӱ��ȫ������������ֲ������ٶ�
    double *posNew;
    int times2;               // ѭ��������
    int mode;                 // ѡ�񷽳�
	CostFactorPyramid* cost_factor_pyramid;
	double init_x[6];
	double up_PSO[6];
	double down_PSO[6];
public:
	PSO(Mat &cam_img_input, double* init_pos, double * init_quat) 
	:MatchSolver(cam_img_input)
	{	
		particle = NULL;
		cost_factor_pyramid = new CostFactorPyramid(cam_img_input, 0);//��ʱ���ý���������
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
		//pn ����������dά��
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
        dim = d;//ά��
		particle = NULL;
       // particle = new Particle[pn];         // ������������ʵ������

      //  for( int i=0; i<pn; ++i )
     //   {
    //        particle[i].setDim(d);           // �������ӵ�ά��
     //   }
    }

    double function( double *x );               //  ����⺯��
    void initial();
    void createNew(int n);                         // �����µ������ٶȺ�λ��
    bool isExited(int n);
	void doPSO(double * p_best, double &score_best);                          //
    void print( double *x );
    void setPNum(int n){ pNum = n;};           // ������������
    void setDim( int d){ dim = d;};           // ����ά��

};
