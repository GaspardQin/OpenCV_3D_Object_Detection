#include "dlib\optimization.h"
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
//using namespace dlib;

typedef dlib::matrix<double, 6, 1> 
;
typedef dlib::matrix<double, 6, 1> parameter_delta_vector;
typedef dlib::matrix<double, 1, 6> jac_vector;
template <typename Fun,
	      typename FunJac>
class LM {
public:
	const Fun* fun;
	const FunJac* fun_jac;
	parameter_vector params_final;
	parameter_vector params;
	double lambda, rho;
	double epsilon_thresh;
	double epsilon;
	LM(const Fun &fun_, const FunJac &fun_jac_, parameter_vector &params_, double lambda_0_, double rho_, double epsilon_thresh_) {
		fun = &fun_;//�ܷ���f(x)ֵ�ĺ���
		fun_jac = &fun_jac_;//�ܷ����ſɱȾ���ĺ���
		lambda = lambda_0_;//����ϵ��
		rho = rho_;//�������ӻ���С�����ı���ϵ��
		epsilon_thresh = epsilon_thresh_;//����
		params = params_;//��ʼֵ
	
	}
	bool LM_solver(parameter_vector &params_final,double &epsilon_final) {
		int max_iteral = 500000;
		int i=0;
		double epsilon_new=100000;
		jac_vector jac;
		parameter_delta_vector delta;
		epsilon = fun->calculateFactor(params);
		jac = fun_jac->calculateDer(params);
		double T = 100000;
		double T_rho = 2;
		while (i < max_iteral && epsilon_new > epsilon_thresh) {
			
			delta = dlib::inv(dlib::trans(jac)* jac
				+ lambda * dlib::identity_matrix<double>(6))*dlib::trans(jac)*epsilon;//����
			std::cout << "delta " << delta << std::endl;
			epsilon_new = fun->calculateFactor(params - delta);
			if (epsilon_new < epsilon) {
				epsilon = epsilon_new;
				lambda /= rho;
				params = params - delta;
				jac = fun_jac->calculateDer(params);
			}
			else {
				if (lambda > 256) {
					srand((unsigned)time(NULL));
					if (exp((epsilon - epsilon_new) / T) > rand() / (RAND_MAX + 1)) {
						params = params - delta;
						jac = fun_jac->calculateDer(params);
						lambda = 0.25;
						epsilon = epsilon_new;
						T = 2;
					}
					

					if (lambda > 10000) {
						//��ʱ������ֲ���Сֵ,����jac����Ӧʹ������
						epsilon = epsilon_new;
						params = params - delta;
						jac = fun_jac->calculateDerPrecise(params);
						lambda = 0.5;

					}
				}
				else {
					lambda *= rho;
				}
			}
			std::cout << "rho: " << lambda << endl;
			i++;
		}
	
		
		if (epsilon_new < epsilon_thresh) {
			params_final = params + delta;
			epsilon_final = epsilon_new;
			return true;
		}
		else if (i = max_iteral) {
			std::cout << "can't solve in " << max_iteral << " iterals" << std::endl;
			return false;
		}
	}
};