#pragma once
#include "PSO.h"
// 目标函数
double PSO::function(double *x)
{

	return this->hausdorffCost(x);

}

// 初始化
void PSO::initial()
{

	c1 = 2.05;
	c2 = 2.05;
	fg = MAX_PSO;                                  // 全局最小寄存量
	bestIndex = 0;                             // 记录最好粒子的序号                 
	int i, j, k;
	times2 = 0;

	static int kk = (unsigned)time(NULL);    // 随机函数的时间种子
	srand((unsigned)time(NULL) + kk++);

	if (particle == NULL)
	{
		particle = new Particle[pNum];           // 创建粒子数组实例对象

		for (i = 0; i<pNum; ++i)
		{
			particle[i].setDim(dim);             // 设置粒子的维数
												 // 分配内存空间
			particle[i].position = new double[dim];
			particle[i].velocity = new double[dim];
			// 初始化粒子的位置
			for (j = 0; j<dim; ++j)
			{
				particle[i].position[j] = rand() / (double)RAND_MAX*UP_PSO;
			}
			// 初始化粒子的速度
			for (j = 0; j<dim; ++j)
			{
				particle[i].velocity[j] = rand() / (double)RAND_MAX*UP_PSO;
			}
		}
	}

	p = new double *[pNum];                      // 记录局部最优的位置
	posNew = new double[dim];

	for (i = 0; i<pNum; ++i)
	{
		p[i] = new double[dim];
		for (j = 0; j<dim; ++j)
			p[i][j] = particle[i].position[j];          // 初始化局部最优位置

														// 计算出示条件下的粒子适应度
		particle[i].fit = function(particle[i].position);

		// 获取全局最小
		if (particle[i].fit<fg)
		{
			fg = particle[i].fit;
			bestIndex = i;                             // 改变全局最优 的索引位置
		}
	}
} // end function initial

  // 更新粒子的速度、和位置
void PSO::createNew(int n)
{
	int i, j, k;

	static int kk = (unsigned)time(NULL);           // 随机函数的时间种子
	srand((unsigned)time(NULL) + kk++);
	// 更新速度
	double b = c1 + c2;
	double q = 2.0 / fabs(2.0 - b - sqrt(b*b - 4 * b));          // 计算压缩因子

	for (i = 0; i<pNum; ++i)
	{
		for (j = 0; j<particle[i].dim; ++j)
		{
			w = WMIN_PSO + (WMAX_PSO - WMIN_PSO)*n / CNT_PSO;            //  计算惯性权重

																		 // 更新粒子的速度
			particle[i].velocity[j] = w*particle[i].velocity[j] +
				rand() / (double)RAND_MAX * c1 *(p[i][j] - particle[i].position[j]) +
				rand() / (double)RAND_MAX * c2 * (p[bestIndex][j] - particle[i].position[j]);
			particle[i].velocity[j] *= q;
			// 控制速度的范围
			if (particle[i].velocity[j]>UP_PSO)
				particle[i].velocity[j] = UP_PSO;
			else if (particle[i].velocity[j]<DOWN_PSO)
				particle[i].velocity[j] = DOWN_PSO;
		}
	}

	// 更新位置
	for (i = 0; i<pNum; ++i)
	{
		for (j = 0; j<particle[i].dim; ++j)
		{
			particle[i].position[j] += particle[i].velocity[j];
			// 控制粒子的位置范围
			if (particle[i].position[j]>UP_PSO)
				particle[i].position[j] = UP_PSO;
			else if (particle[i].position[j]<DOWN_PSO)
				particle[i].position[j] = DOWN_PSO;
		}

		// 计算该位置的函数值 （适应度）
		double tf = function(particle[i].position);

		// 检查是否局部最优
		if (tf<particle[i].fit)
		{
			particle[i].fit = tf;
			for (k = 0; k<particle[i].dim; ++k)
				p[i][k] = particle[i].position[k];

			// 检查是否全局最优
			if (tf<fg)
			{
				lastFG = fg;
				fg = tf;
				bestIndex = i;
			}
		}

		// 进行随机摄动
		for (j = 0; j<particle[i].dim; ++j)
		{
			posNew[j] = particle[i].position[j] + STEP_PSO*rand() / (double)RAND_MAX;
		}

		tf = function(posNew);
		if (tf>particle[i].fit)
		{
			for (j = 0; j<particle[i].dim; ++j)
			{
				particle[i].position[j] = posNew[j];
			}
		}
	}
}

// 检查是否满足退出条件
bool PSO::isExited(int n)
{
	if (n>CNT_PSO || fabs(fg - lastFG) / fg<e_PSO)
		return true;

	return false;
}

// 粒子群算法求解
void PSO::doPSO(double * p_best, double &output_best)
{
	int n = 0;                    // 整个搜索过程的迭代计算机
	int i;
	initial();            // 初始化
						  //粒子群算法进行最优值搜索
	do
	{
		createNew(n);
		++n;
		// 如果times2大于等于指定的时代，则对最差粒子重新初始化其位置和速度
		++times2;
		if (times2 >= 10)
		{
			times2 = 0;
			int tempIndex = 0;                  // 定义并初始化最差粒子的序号
			double worse = particle[0].fit;                // 定义并初始化最差粒子的适应度
														   // 搜索最差粒子
			for (i = 1; i<pNum; ++i)
			{
				if (worse>particle[i].fit)
				{
					worse = particle[i].fit;
					tempIndex = i;
				}
			}
			static int kk = (unsigned)time(NULL);    // 随机函数的时间种子
			srand((unsigned)time(NULL) + kk++);
			// 对最差粒子重新初始化其位置和速度
			for (i = 0; i<particle[tempIndex].dim; ++i)
			{
				particle[i].position[tempIndex] = rand() / (double)RAND_MAX*UP_PSO;
				particle[i].velocity[tempIndex] = rand() / (double)RAND_MAX*UP_PSO;
			}
		}
	} while (!isExited(n));
	p_best = p[bestIndex];
	output_best = fg;
	// 输出最终结果
	//cout<<"最优值点： "<<endl;
	//cout<<"\t";
	//print(p[bestIndex]);
	//cout.precision(10);
	//cout<<"最优值:  "<<fixed<<fg<<endl;
}

// 输出相关结果
void PSO::print(double *x)
{
	for (int i = 0; i<dim; i++)
	{
		cout << x[i] << "   ";
	}
	cout << endl;
}
