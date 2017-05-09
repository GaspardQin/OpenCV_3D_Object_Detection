//该类封装了 ceres提供的非线性最优化方法
#pragma once
#include <MatchEdges.h>
#define GLOG_NO_ABBREVIATED_SEVERITIES
#include <ceres\ceres.h>

class MatchSolver {
public:
	ceres::Problem problem;

	void setIniVar(double x, double y, double z, double x_deg, double y_deg, double z_deg);


	class NumericDiffCostFunctor:public MatchEdges	
	{
	public:
		NumericDiffCostFunctor(Mat *cam_img_input)
			:MatchEdges(cam_img_input)
		{};
		bool operator()(const double* const var, double* residual) const {
			//var 代表6维度输入参数，(x,y,z,yaw,pitch,roll)
			residual[0] = hausdorffDistance(var);
			
			return true;
		}
	};

	void solve(Mat *cam_img_input, double* final_result);
	

private:
	double var[6];
	
};

void MatchSolver::setIniVar(double x, double y, double z, double x_deg, double y_deg, double z_deg) {
	var[0] = x;
	var[1] = y;
	var[2] = z;
	var[3] = x_deg;
	var[4] = y_deg;
	var[5] = z_deg;

}
void MatchSolver::solve(Mat *cam_img_input, double* final_result) {
	google::InitGoogleLogging("solve");
	ceres::CostFunction* cost_function
		= new ceres::NumericDiffCostFunction<NumericDiffCostFunctor, ceres::CENTRAL, 1, 6 >(
			new NumericDiffCostFunctor(cam_img_input));                   // |                                                           |              | | 
		                              //        Finite Differencing Scheme --+        | | 
		                            //    Dimension of residual---------------------- + | 
		                          //   Dimension of var-------------------------------- + 

	problem.AddResidualBlock(cost_function, NULL ,var);

	//开始最优化
	ceres::Solver::Options options;
	options.minimizer_type = ceres::TRUST_REGION;
	options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
	options.minimizer_progress_to_stdout = true;
	options.max_num_iterations = 50;
	options.num_threads = 1; //多线程时必须优化OpenGL的同步问题
	options.min_trust_region_radius = 1e-3;//最小可信域，相当于误差阈值
	
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	std::cout << "The final position and rotation are:  x:" << var[0] << " y: " << var[1] << " z: " << var[2] << " x_deg:" << var[3] << " y_deg:" << var[4] << " z_deg:" << var[5] << std::endl;
	
	final_result = var;
}
