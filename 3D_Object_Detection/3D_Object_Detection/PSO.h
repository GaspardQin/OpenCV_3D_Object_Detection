#include"particle.h"

#include <iostream>
#include <cmath>
#include <ctime>
#include <cstdlib>

using namespace std;

#define  e 0.000001
#define CNT 30000
#define UP 10
#define DOWN -5
#define MAX 999999
#define WMAX 1.1
#define WMIN 0.9
#define STEP 1.0                    // ����㶯����
#define PI 3.1415926

class PSO
{
protected:
    Particle *particle;             // ����
    int pNum;                       // ��������
    int dim;                        // ά��
    double **p;                     // �ֲ����� ĳһ�����������̵�����λ��
    int bestIndex;                  // ȫ�����Ŷ�Ӧ�����ӵ�����ֵ
    double fg;                      // ȫ������
    double lastFG;                  // ǰһ�������̵�ȫ������
    double c1;
    double c2;
    double w;
    double *posNew;
    int times2;               // ѭ��������
    int mode;                 // ѡ�񷽳�
public:
    PSO(){particle=NULL;};
    PSO( int pn, int d )
    {
        pNum = pn;
        dim = d;

        particle = new Particle[pn];         // ������������ʵ������

        for( int i=0; i<pn; ++i )
        {
            particle[i].setDim(d);           // �������ӵ�ά��
        }
    }

    double function( double *x );               //  ����⺯��
    void initial(int pn);
    void createNew(int n);                         // �����µ������ٶȺ�λ��
    bool isExited(int n);
    void doPSO(int pa);                             //
    void print( double *x );
    void setPNum(int n){ pNum = n;};           // ������������
    void setDim( int d){ dim = d;};           // ����ά��

};

// Ŀ�꺯��
double PSO::function( double *x )
{
    double r;

    if(mode==1 )
    {
        r =100 * (x[1] -x[0]*x[0]) * (x[1] -x[0]*x[0]) + 1 -
	        	2 * x[0] + x[0]*x[0];
    }
    else if( mode == 2 )
    {

        r = 100*(x[0]*x[0]-x[1])*(x[0]*x[0]-x[1]) + (x[0]-1)*(x[0]-1) +
                (x[2]-1)*(x[2]-1) + 90*(x[2]*x[2]-x[3])*(x[2]*x[2]-x[3]) +
                10.1*(x[1]-1)*(x[1]-1) + 10.1*(x[3]-1)*(x[3]-1)+19.8*(x[1]-1)*(x[3]-1);
    }
    else if( mode==3 )
    {
        r = (x[1]-5.1*x[0]*x[0]/(4*PI*PI)+5.0*x[0]/PI-6.0)*(x[1]-5.1*x[0]*x[0]/(4*PI*PI)+5.0*x[0]/PI-6.0)+
                10*( 1-1.0/(8.0*PI) )*cos(x[0])+10;
    }


    return r;
}

// ��ʼ��
void PSO::initial(int pn)
{

    cout<<"--------------------------------------------------------"<<endl;
    cout<<"------------------����㶯����Ⱥ�㷨--------------------"<<endl;
    cout<<"--------------------------------------------------------"<<endl;

    cout<<"���Ժ��� 1 "<<endl;
    cout<<"100*(x2-x1^2)^2+1-2*x1+x1^2"<<endl;
    cout<<endl;
    cout<<"���Ժ��� 2 "<<endl;
    cout<<"100*(x1^2-x2)^2+(x1-1)^2+(x3-1)^2+ 90*(x3^2-x4)^2+";
    cout<<"10.1*(x2-1)^2+ 10.1*(x4-1)^2+19.8*(x2-1)*(x4-1)"<<endl;
    cout<<endl;

    cout<<"���Ժ��� 3"<<endl;
    cout<<"(x2-5.1*x1^2/4*PI^2+5*x1/PI-6)^2+10*(1-1/8*PI)cosx1+10"<<endl;
    cout<<endl;

    cout<<"��ѡ��Ŀ�꺯�� 1  2  3"<<endl;
    cin>>mode;

    switch( mode )
    {
        case 1:
        case 3:
            dim=2;
            break;
        case 2:
            dim=4;
            break;
    }
    c1 = 2.05;
    c2 = 2.05;
    fg = MAX;                                  // ȫ������
    bestIndex = 0;                             // ��¼������ӵ����
    pNum = pn;                                 // ������������Ⱥ��С��
    int i,j,k;
    times2=0;

    static int kk = (unsigned) time (NULL);    // ���������ʱ������
	srand( (unsigned)time(NULL) + kk++ );

    if( particle==NULL )
    {
        particle = new Particle[pn];           // ������������ʵ������

        for( i=0; i<pn; ++i )
        {
            particle[i].setDim(dim);             // �������ӵ�ά��
            // �����ڴ�ռ�
            particle[i].position = new double[dim];
            particle[i].velocity = new double[dim];
            // ��ʼ�����ӵ�λ��
            for( j=0; j<dim; ++j )
            {
                particle[i].position[j]=rand()/(double)RAND_MAX*UP;
            }
            // ��ʼ�����ӵ��ٶ�
            for( j=0; j<dim; ++j )
            {
                particle[i].velocity[j]=rand()/(double)RAND_MAX*UP;
            }
        }
    }

    p = new double *[pn];                      // ��¼�ֲ����ŵ�λ��
    posNew = new double [dim];

    for( i=0; i<pn; ++i )
    {
        p[i] = new double[dim];
        for( j=0; j<dim; ++j )
            p[i][j] = particle[i].position[j];          // ��ʼ���ֲ�����λ��

        // �����ʾ�����µ�������Ӧ��
        particle[i].fit  = function( particle[i].position );

        // ��ȡȫ����С
        if( particle[i].fit<fg )
        {
            fg= particle[i].fit;
            bestIndex = i;                             // �ı�ȫ������ ������λ��
        }
    }
} // end function initial

// �������ӵ��ٶȡ���λ��
void PSO::createNew(int n)
{
    int i,j,k;

    static int kk = (unsigned) time (NULL);           // ���������ʱ������
	srand( (unsigned)time(NULL) + kk++ );
    // �����ٶ�
    double b=c1+c2;
    double q=2.0/fabs(2.0-b-sqrt(b*b-4*b) );          // ����ѹ������

    for( i=0; i<pNum; ++i )
    {
        for( j=0; j<particle[i].dim; ++j )
        {
            w = WMIN + (WMAX-WMIN)*n/CNT;            //  �������Ȩ��

            // �������ӵ��ٶ�
            particle[i].velocity[j] = w*particle[i].velocity[j] +
                                        rand()/(double)RAND_MAX * c1 *( p[i][j]-particle[i].position[j] ) +
                                        rand()/(double)RAND_MAX * c2 * (p[bestIndex][j]-particle[i].position[j]);
            particle[i].velocity[j] *=q;
            // �����ٶȵķ�Χ
            if( particle[i].velocity[j]>UP )
                particle[i].velocity[j]=UP;
            else if( particle[i].velocity[j]<DOWN )
                particle[i].velocity[j]=DOWN;
        }
    }

    // ����λ��
    for( i=0; i<pNum; ++i )
    {
        for( j=0; j<particle[i].dim; ++j )
        {
            particle[i].position[j] += particle[i].velocity[j];
            // �������ӵ�λ�÷�Χ
            if( particle[i].position[j]>UP )
                particle[i].position[j]=UP;
            else if( particle[i].position[j]<DOWN )
                particle[i].position[j]=DOWN;
        }

        // �����λ�õĺ���ֵ ����Ӧ�ȣ�
        double tf = function( particle[i].position );

        // ����Ƿ�ֲ�����
        if( tf<particle[i].fit )
        {
            particle[i].fit = tf;
            for( k=0; k<particle[i].dim; ++k )
                p[i][k] = particle[i].position[k];

            // ����Ƿ�ȫ������
            if( tf<fg )
            {
                lastFG = fg;
                fg = tf;
                bestIndex = i;
            }
        }

        // ��������㶯
        for( j=0; j<particle[i].dim; ++j )
        {
            posNew[j]= particle[i].position[j]+STEP*rand()/(double)RAND_MAX ;
        }

        tf = function( posNew );
        if(tf>particle[i].fit )
        {
             for( j=0; j<particle[i].dim; ++j )
             {
                 particle[i].position[j]=posNew[j];
             }
        }
    }
}

// ����Ƿ������˳�����
bool PSO::isExited( int n )
{
    if( n>CNT || fabs(fg-lastFG)/fg<e )
        return true;

    return false;
}

// ����Ⱥ�㷨���
void PSO::doPSO(int pn)
{
    int n=0;                    // �����������̵ĵ��������
    int i;
    initial(pn);            // ��ʼ��
    //����Ⱥ�㷨��������ֵ����
    do
    {
        createNew(n);
        ++n;
        // ���times2���ڵ���ָ����ʱ�����������������³�ʼ����λ�ú��ٶ�
        ++times2;
        if( times2>=10 )
        {
            times2=0;
            int tempIndex=0;                  // ���岢��ʼ��������ӵ����
            double worse= particle[0].fit;                // ���岢��ʼ��������ӵ���Ӧ��
            // �����������
            for( i=1; i<pNum; ++i )
            {
                if( worse>particle[i].fit )
                {
                    worse = particle[i].fit;
                    tempIndex=i;
                }
            }
            static int kk = (unsigned) time (NULL);    // ���������ʱ������
	        srand( (unsigned)time(NULL) + kk++ );
	        // ������������³�ʼ����λ�ú��ٶ�
            for( i=0; i<particle[tempIndex].dim; ++i )
            {
                particle[i].position[tempIndex]=rand()/(double)RAND_MAX*UP;
                particle[i].velocity[tempIndex]=rand()/(double)RAND_MAX*UP;
            }
        }
    }while( !isExited(n) );
    // ������ս��
    cout<<"����ֵ�㣺 "<<endl;
    cout<<"\t";
    print(p[bestIndex]);
    cout.precision(10);
    cout<<"����ֵ:  "<<fixed<<fg<<endl;
}

// �����ؽ��
void PSO::print(double *x )
{
    for( int i=0; i<dim; i++ )
    {
        cout<<x[i]<<"   ";
    }
    cout<<endl;
}
