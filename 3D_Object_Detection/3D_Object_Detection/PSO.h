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
#define STEP_PSO 1.0                    // ����㶯����
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
public:
	PSO(Mat &cam_img_input, double* init_pos, double * init_quat) 
	:MatchSolver(cam_img_input)
	{	
		particle = NULL;
	};
    PSO(Mat &cam_img_input, double* init_pos, double* init_quat, int pn, int d )
		//pn ����������dά��
		:MatchSolver(cam_img_input)
    {
		
        pNum = pn;
        dim = d;//ά��

        particle = new Particle[pn];         // ������������ʵ������

        for( int i=0; i<pn; ++i )
        {
            particle[i].setDim(d);           // �������ӵ�ά��
        }
    }

    double function( double *x );               //  ����⺯��
    void initial();
    void createNew(int n);                         // �����µ������ٶȺ�λ��
    bool isExited(int n);
	void doPSO(double * p_best, double &output_best);                           //
    void print( double *x );
    void setPNum(int n){ pNum = n;};           // ������������
    void setDim( int d){ dim = d;};           // ����ά��

};
