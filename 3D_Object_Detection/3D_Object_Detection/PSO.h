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
#define STEP 1.0                    // 随机摄动步长
#define PI 3.1415926

class PSO
{
protected:
    Particle *particle;             // 粒子
    int pNum;                       // 粒子数量
    int dim;                        // 维数
    double **p;                     // 局部最优 某一粒子整个过程的最优位置
    int bestIndex;                  // 全局最优对应的粒子的索引值
    double fg;                      // 全局最优
    double lastFG;                  // 前一迭代过程的全局最优
    double c1;
    double c2;
    double w;
    double *posNew;
    int times2;               // 循环计数器
    int mode;                 // 选择方程
public:
    PSO(){particle=NULL;};
    PSO( int pn, int d )
    {
        pNum = pn;
        dim = d;

        particle = new Particle[pn];         // 创建粒子数组实例对象

        for( int i=0; i<pn; ++i )
        {
            particle[i].setDim(d);           // 设置粒子的维数
        }
    }

    double function( double *x );               //  待求解函数
    void initial(int pn);
    void createNew(int n);                         // 生成新的粒子速度和位置
    bool isExited(int n);
    void doPSO(int pa);                             //
    void print( double *x );
    void setPNum(int n){ pNum = n;};           // 设置粒子数量
    void setDim( int d){ dim = d;};           // 设置维数

};

// 目标函数
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

// 初始化
void PSO::initial(int pn)
{

    cout<<"--------------------------------------------------------"<<endl;
    cout<<"------------------随机摄动粒子群算法--------------------"<<endl;
    cout<<"--------------------------------------------------------"<<endl;

    cout<<"测试函数 1 "<<endl;
    cout<<"100*(x2-x1^2)^2+1-2*x1+x1^2"<<endl;
    cout<<endl;
    cout<<"测试函数 2 "<<endl;
    cout<<"100*(x1^2-x2)^2+(x1-1)^2+(x3-1)^2+ 90*(x3^2-x4)^2+";
    cout<<"10.1*(x2-1)^2+ 10.1*(x4-1)^2+19.8*(x2-1)*(x4-1)"<<endl;
    cout<<endl;

    cout<<"测试函数 3"<<endl;
    cout<<"(x2-5.1*x1^2/4*PI^2+5*x1/PI-6)^2+10*(1-1/8*PI)cosx1+10"<<endl;
    cout<<endl;

    cout<<"请选择目标函数 1  2  3"<<endl;
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
    fg = MAX;                                  // 全局最优
    bestIndex = 0;                             // 记录最好粒子的序号
    pNum = pn;                                 // 粒子数量（种群大小）
    int i,j,k;
    times2=0;

    static int kk = (unsigned) time (NULL);    // 随机函数的时间种子
	srand( (unsigned)time(NULL) + kk++ );

    if( particle==NULL )
    {
        particle = new Particle[pn];           // 创建粒子数组实例对象

        for( i=0; i<pn; ++i )
        {
            particle[i].setDim(dim);             // 设置粒子的维数
            // 分配内存空间
            particle[i].position = new double[dim];
            particle[i].velocity = new double[dim];
            // 初始化粒子的位置
            for( j=0; j<dim; ++j )
            {
                particle[i].position[j]=rand()/(double)RAND_MAX*UP;
            }
            // 初始化粒子的速度
            for( j=0; j<dim; ++j )
            {
                particle[i].velocity[j]=rand()/(double)RAND_MAX*UP;
            }
        }
    }

    p = new double *[pn];                      // 记录局部最优的位置
    posNew = new double [dim];

    for( i=0; i<pn; ++i )
    {
        p[i] = new double[dim];
        for( j=0; j<dim; ++j )
            p[i][j] = particle[i].position[j];          // 初始化局部最优位置

        // 计算出示条件下的粒子适应度
        particle[i].fit  = function( particle[i].position );

        // 获取全局最小
        if( particle[i].fit<fg )
        {
            fg= particle[i].fit;
            bestIndex = i;                             // 改变全局最优 的索引位置
        }
    }
} // end function initial

// 更新粒子的速度、和位置
void PSO::createNew(int n)
{
    int i,j,k;

    static int kk = (unsigned) time (NULL);           // 随机函数的时间种子
	srand( (unsigned)time(NULL) + kk++ );
    // 更新速度
    double b=c1+c2;
    double q=2.0/fabs(2.0-b-sqrt(b*b-4*b) );          // 计算压缩因子

    for( i=0; i<pNum; ++i )
    {
        for( j=0; j<particle[i].dim; ++j )
        {
            w = WMIN + (WMAX-WMIN)*n/CNT;            //  计算惯性权重

            // 更新粒子的速度
            particle[i].velocity[j] = w*particle[i].velocity[j] +
                                        rand()/(double)RAND_MAX * c1 *( p[i][j]-particle[i].position[j] ) +
                                        rand()/(double)RAND_MAX * c2 * (p[bestIndex][j]-particle[i].position[j]);
            particle[i].velocity[j] *=q;
            // 控制速度的范围
            if( particle[i].velocity[j]>UP )
                particle[i].velocity[j]=UP;
            else if( particle[i].velocity[j]<DOWN )
                particle[i].velocity[j]=DOWN;
        }
    }

    // 更新位置
    for( i=0; i<pNum; ++i )
    {
        for( j=0; j<particle[i].dim; ++j )
        {
            particle[i].position[j] += particle[i].velocity[j];
            // 控制粒子的位置范围
            if( particle[i].position[j]>UP )
                particle[i].position[j]=UP;
            else if( particle[i].position[j]<DOWN )
                particle[i].position[j]=DOWN;
        }

        // 计算该位置的函数值 （适应度）
        double tf = function( particle[i].position );

        // 检查是否局部最优
        if( tf<particle[i].fit )
        {
            particle[i].fit = tf;
            for( k=0; k<particle[i].dim; ++k )
                p[i][k] = particle[i].position[k];

            // 检查是否全局最优
            if( tf<fg )
            {
                lastFG = fg;
                fg = tf;
                bestIndex = i;
            }
        }

        // 进行随机摄动
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

// 检查是否满足退出条件
bool PSO::isExited( int n )
{
    if( n>CNT || fabs(fg-lastFG)/fg<e )
        return true;

    return false;
}

// 粒子群算法求解
void PSO::doPSO(int pn)
{
    int n=0;                    // 整个搜索过程的迭代计算机
    int i;
    initial(pn);            // 初始化
    //粒子群算法进行最优值搜索
    do
    {
        createNew(n);
        ++n;
        // 如果times2大于等于指定的时代，则对最差粒子重新初始化其位置和速度
        ++times2;
        if( times2>=10 )
        {
            times2=0;
            int tempIndex=0;                  // 定义并初始化最差粒子的序号
            double worse= particle[0].fit;                // 定义并初始化最差粒子的适应度
            // 搜索最差粒子
            for( i=1; i<pNum; ++i )
            {
                if( worse>particle[i].fit )
                {
                    worse = particle[i].fit;
                    tempIndex=i;
                }
            }
            static int kk = (unsigned) time (NULL);    // 随机函数的时间种子
	        srand( (unsigned)time(NULL) + kk++ );
	        // 对最差粒子重新初始化其位置和速度
            for( i=0; i<particle[tempIndex].dim; ++i )
            {
                particle[i].position[tempIndex]=rand()/(double)RAND_MAX*UP;
                particle[i].velocity[tempIndex]=rand()/(double)RAND_MAX*UP;
            }
        }
    }while( !isExited(n) );
    // 输出最终结果
    cout<<"最优值点： "<<endl;
    cout<<"\t";
    print(p[bestIndex]);
    cout.precision(10);
    cout<<"最优值:  "<<fixed<<fg<<endl;
}

// 输出相关结果
void PSO::print(double *x )
{
    for( int i=0; i<dim; i++ )
    {
        cout<<x[i]<<"   ";
    }
    cout<<endl;
}
