#include "dlib\optimization.h"
//using namespace dlib;

typedef dlib::matrix<double, 6, 1> parameter_vector;
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
		int max_iteral = 50;
		int i=0;
		double epsilon_new=100000;
		jac_vector jac;
		parameter_delta_vector delta;
		epsilon = fun->calculateFactor(params);
		jac = fun_jac->calculateDer(params);
		while (i < max_iteral && epsilon_new > epsilon_thresh) {
			
			delta = (dlib::inv(dlib::trans(jac)* jac
				+ lambda * dlib::identity_matrix<double>(6))*dlib::trans(jac)*epsilon)*epsilon;//����
			epsilon_new = fun->calculateFactor(params - delta);
			if (epsilon_new < epsilon) {
				epsilon = epsilon_new;
				lambda /= rho;
				params = params - delta;
				jac = fun_jac->calculateDer(params);
			}
			else {
				lambda *= rho;

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