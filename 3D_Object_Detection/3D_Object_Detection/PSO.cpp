#pragma once
#include "PSO.h"
// Ŀ�꺯��
double PSO::function(double *x)
{

	return this->hausdorffCost(x);

}

// ��ʼ��
void PSO::initial()
{

	c1 = 2.05;
	c2 = 2.05;
	fg = MAX_PSO;                                  // ȫ����С�Ĵ���
	bestIndex = 0;                             // ��¼������ӵ����                 
	int i, j, k;
	times2 = 0;

	static int kk = (unsigned)time(NULL);    // ���������ʱ������
	srand((unsigned)time(NULL) + kk++);

	if (particle == NULL)
	{
		particle = new Particle[pNum];           // ������������ʵ������

		for (i = 0; i<pNum; ++i)
		{
			particle[i].setDim(dim);             // �������ӵ�ά��
												 // �����ڴ�ռ�
			particle[i].position = new double[dim];
			particle[i].velocity = new double[dim];
			// ��ʼ�����ӵ�λ��
			for (j = 0; j<dim; ++j)
			{
				particle[i].position[j] = rand() / (double)RAND_MAX*UP_PSO;
			}
			// ��ʼ�����ӵ��ٶ�
			for (j = 0; j<dim; ++j)
			{
				particle[i].velocity[j] = rand() / (double)RAND_MAX*UP_PSO;
			}
		}
	}

	p = new double *[pNum];                      // ��¼�ֲ����ŵ�λ��
	posNew = new double[dim];

	for (i = 0; i<pNum; ++i)
	{
		p[i] = new double[dim];
		for (j = 0; j<dim; ++j)
			p[i][j] = particle[i].position[j];          // ��ʼ���ֲ�����λ��

														// �����ʾ�����µ�������Ӧ��
		particle[i].fit = function(particle[i].position);

		// ��ȡȫ����С
		if (particle[i].fit<fg)
		{
			fg = particle[i].fit;
			bestIndex = i;                             // �ı�ȫ������ ������λ��
		}
	}
} // end function initial

  // �������ӵ��ٶȡ���λ��
void PSO::createNew(int n)
{
	int i, j, k;

	static int kk = (unsigned)time(NULL);           // ���������ʱ������
	srand((unsigned)time(NULL) + kk++);
	// �����ٶ�
	double b = c1 + c2;
	double q = 2.0 / fabs(2.0 - b - sqrt(b*b - 4 * b));          // ����ѹ������

	for (i = 0; i<pNum; ++i)
	{
		for (j = 0; j<particle[i].dim; ++j)
		{
			w = WMIN_PSO + (WMAX_PSO - WMIN_PSO)*n / CNT_PSO;            //  �������Ȩ��

																		 // �������ӵ��ٶ�
			particle[i].velocity[j] = w*particle[i].velocity[j] +
				rand() / (double)RAND_MAX * c1 *(p[i][j] - particle[i].position[j]) +
				rand() / (double)RAND_MAX * c2 * (p[bestIndex][j] - particle[i].position[j]);
			particle[i].velocity[j] *= q;
			// �����ٶȵķ�Χ
			if (particle[i].velocity[j]>UP_PSO)
				particle[i].velocity[j] = UP_PSO;
			else if (particle[i].velocity[j]<DOWN_PSO)
				particle[i].velocity[j] = DOWN_PSO;
		}
	}

	// ����λ��
	for (i = 0; i<pNum; ++i)
	{
		for (j = 0; j<particle[i].dim; ++j)
		{
			particle[i].position[j] += particle[i].velocity[j];
			// �������ӵ�λ�÷�Χ
			if (particle[i].position[j]>UP_PSO)
				particle[i].position[j] = UP_PSO;
			else if (particle[i].position[j]<DOWN_PSO)
				particle[i].position[j] = DOWN_PSO;
		}

		// �����λ�õĺ���ֵ ����Ӧ�ȣ�
		double tf = function(particle[i].position);

		// ����Ƿ�ֲ�����
		if (tf<particle[i].fit)
		{
			particle[i].fit = tf;
			for (k = 0; k<particle[i].dim; ++k)
				p[i][k] = particle[i].position[k];

			// ����Ƿ�ȫ������
			if (tf<fg)
			{
				lastFG = fg;
				fg = tf;
				bestIndex = i;
			}
		}

		// ��������㶯
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

// ����Ƿ������˳�����
bool PSO::isExited(int n)
{
	if (n>CNT_PSO || fabs(fg - lastFG) / fg<e_PSO)
		return true;

	return false;
}

// ����Ⱥ�㷨���
void PSO::doPSO(double * p_best, double &output_best)
{
	int n = 0;                    // �����������̵ĵ��������
	int i;
	initial();            // ��ʼ��
						  //����Ⱥ�㷨��������ֵ����
	do
	{
		createNew(n);
		++n;
		// ���times2���ڵ���ָ����ʱ�����������������³�ʼ����λ�ú��ٶ�
		++times2;
		if (times2 >= 10)
		{
			times2 = 0;
			int tempIndex = 0;                  // ���岢��ʼ��������ӵ����
			double worse = particle[0].fit;                // ���岢��ʼ��������ӵ���Ӧ��
														   // �����������
			for (i = 1; i<pNum; ++i)
			{
				if (worse>particle[i].fit)
				{
					worse = particle[i].fit;
					tempIndex = i;
				}
			}
			static int kk = (unsigned)time(NULL);    // ���������ʱ������
			srand((unsigned)time(NULL) + kk++);
			// ������������³�ʼ����λ�ú��ٶ�
			for (i = 0; i<particle[tempIndex].dim; ++i)
			{
				particle[i].position[tempIndex] = rand() / (double)RAND_MAX*UP_PSO;
				particle[i].velocity[tempIndex] = rand() / (double)RAND_MAX*UP_PSO;
			}
		}
	} while (!isExited(n));
	p_best = p[bestIndex];
	output_best = fg;
	// ������ս��
	//cout<<"����ֵ�㣺 "<<endl;
	//cout<<"\t";
	//print(p[bestIndex]);
	//cout.precision(10);
	//cout<<"����ֵ:  "<<fixed<<fg<<endl;
}

// �����ؽ��
void PSO::print(double *x)
{
	for (int i = 0; i<dim; i++)
	{
		cout << x[i] << "   ";
	}
	cout << endl;
}
