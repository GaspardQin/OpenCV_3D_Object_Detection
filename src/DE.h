#pragma once

#ifndef _DE_H_
#define _DE_H_
#include "differential_evolution.hpp"
#include "MatchSolver.h"
#include "objective_function.h"
#include "thread_variables.h"
using namespace de;
#define THREAD_NUM 4
#define VARS_COUNT 6
#define POPULATION_SIZE 30
#define LEVEL 0
#define THRESHOLD_FINAL 2000
/**
* Objective function to optimize is "sumDT" 
*/

class DE_factor:public objective_function
{
private:
	boost::shared_ptr<CostFactorPyramid> cost_factor_ptr;
	dlib::matrix<double, 6L, 1L> params;
public:
	DE_factor() :objective_function("sumDT")
	{
		Mat cam_src = imread("../model/sample.jpg", CV_8UC1);
		Mat cam_canny_img;
		Canny(cam_src, cam_canny_img, 50, 200);
		cost_factor_ptr = boost::make_shared<CostFactorPyramid>(cam_canny_img, LEVEL);
	}
	
	virtual double operator()(de::DVectorPtr args)
	{
		/**
		* The two function arguments are the elements index 0 and 1 in
		* the argument vector, as defined by the constraints vector
		* below
		*/
		
		params(0) = (*args)[0];
		params(1) = (*args)[1];
		params(2) = (*args)[2];
		params(3) = (*args)[3];
		params(4) = (*args)[4];
		params(5) = (*args)[5]; 
		return cost_factor_ptr->calculateDTfactorPyramid(params)* pow(2,LEVEL)* pow(2,LEVEL);
	}
};
class DE_factor_offline :public objective_function
{
private:
	boost::shared_ptr<CostFactorDT_OfflineROI> cost_factor_ptr;
	dlib::matrix<double, 6L, 1L> params;
	boost::shared_array<int> buffer_var;
	boost::shared_array<int> buffer_precision;
	boost::shared_array<Mat> model_DT_imgs;
	std::vector<int> vars_valide; std::vector<int> vars_non_valide;
public:
	DE_factor_offline(std::vector<int>& vars_valide_, std::vector<int>& vars_non_valide_, boost::shared_array<Mat>& model_DT_imgs_, boost::shared_array<int>& init_buffer_var, boost::shared_array<int>& init_buffer_precision, boost::shared_array<int>& init_buffer_l_boundary, boost::shared_array<int>& init_buffer_r_boundary, boost::shared_array<int>& init_buffer_count_for_levels) :objective_function("sumDT_offline")
	{
		Mat cam_src = imread("../model/sample.jpg", CV_8UC1);
		Mat cam_canny_img;
		Canny(cam_src, cam_canny_img, 50, 200);
		model_DT_imgs = model_DT_imgs_;
		cost_factor_ptr = boost::make_shared<CostFactorDT_OfflineROI>(cam_canny_img, model_DT_imgs_, init_buffer_l_boundary, init_buffer_r_boundary, init_buffer_precision, init_buffer_count_for_levels);
		buffer_var = init_buffer_var;
		buffer_precision = init_buffer_precision;
		vars_valide = vars_valide_;
		vars_non_valide = vars_non_valide_;
	}

	virtual double operator()(de::DVectorPtr args)
	{
		/**
		* The two function arguments are the elements index 0 and 1 in
		* the argument vector, as defined by the constraints vector
		* below
		*/
		for (int i = 0; i < vars_valide.size(); i++) {
			params(vars_valide[i]) = (*args)[vars_valide[i]] * buffer_precision[vars_valide[i]] + buffer_var[vars_valide[i]];
		}
		for (int i = 0; i < vars_non_valide.size(); i++) {
			params(vars_non_valide[i]) = buffer_var[vars_non_valide[i]];
		}

		return cost_factor_ptr->calculateDTfactorPyramid(params)* pow(2, LEVEL)* pow(2, LEVEL);
	}
};
class DE_factor_offline_modelCanny_camDT :public objective_function
{
private:
	boost::shared_ptr<CostFactorDT_Offline_modelCanny_camDT> cost_factor_ptr;
	dlib::matrix<double, 6L, 1L> params;
	boost::shared_array<int> buffer_var;
	boost::shared_array<int> buffer_precision;
	boost::shared_array<std::vector<Point2i>> model_points_vec_array;
	std::vector<int> vars_valide; std::vector<int> vars_non_valide;
	boost::shared_array<double> cache_match;
public:
	DE_factor_offline_modelCanny_camDT(std::vector<int>& vars_valide_, std::vector<int>& vars_non_valide_, boost::shared_array<std::vector<Point2i>>& model_points_vec_array_, boost::shared_array<int>& init_buffer_var, boost::shared_array<int>& init_buffer_precision, boost::shared_array<int>& init_buffer_l_boundary, boost::shared_array<int>& init_buffer_r_boundary, boost::shared_array<int>& init_buffer_count_for_levels, boost::shared_array<double>& cache_match_) :objective_function("sumDT_offline")
	{
		Mat cam_src = imread("../model/sample.bmp", CV_8UC1);
		Mat cam_canny_img = cam_src;
		//cam_canny_img.convertTo(cam_canny_img, CV_32FC1);
		//Canny(cam_src, cam_canny_img, 50, 200);
		
		model_points_vec_array = model_points_vec_array_;
		cost_factor_ptr = boost::make_shared<CostFactorDT_Offline_modelCanny_camDT>(cam_canny_img, model_points_vec_array, init_buffer_l_boundary, init_buffer_r_boundary, init_buffer_precision, init_buffer_count_for_levels,cache_match_);
		buffer_var = init_buffer_var;
		buffer_precision = init_buffer_precision;
		vars_valide = vars_valide_;
		vars_non_valide = vars_non_valide_;
		cache_match = cache_match_;
	}

	virtual double operator()(de::DVectorPtr args)
	{
		/**
		* The two function arguments are the elements index 0 and 1 in
		* the argument vector, as defined by the constraints vector
		* below
		*/
		for (int i = 0; i < vars_valide.size(); i++) {
			params(vars_valide[i]) = (*args)[vars_valide[i]] * buffer_precision[vars_valide[i]] + buffer_var[vars_valide[i]];
		}
		for (int i = 0; i < vars_non_valide.size(); i++) {
			params(vars_non_valide[i]) = buffer_var[vars_non_valide[i]];
		}

		return cost_factor_ptr->calculateDTfactorPyramid(params)* pow(2, LEVEL)* pow(2, LEVEL);
	}
};


class my_termination_strategy :public de::termination_strategy {
private:
	double threshold;
	size_t maxGen;
public:
	my_termination_strategy(size_t maxGen_, double threshold_) {
		maxGen = maxGen_;
		threshold = threshold_;
	}
	virtual bool event(individual_ptr best, size_t genCount)
	{
		std::cout << "genCount : " << genCount << std::endl;
		return (genCount < maxGen) && (best->cost() > threshold);
	}
};

class DEsolver {
private:
	double init_var[6];
public:
	individual_ptr best;
	DEsolver(double * pos_estimated, double * deg_estimated) {
		init_var[0] = pos_estimated[0];
		init_var[1] = pos_estimated[1];
		init_var[2] = pos_estimated[2];
		init_var[3] = deg_estimated[0];// *rho_quat;
		init_var[4] = deg_estimated[1];// *rho_quat;
		init_var[5] = deg_estimated[2];// *rho_quat;
	}
	void setInitVar(double x, double y, double z, double deg_x, double deg_y, double deg_z) {
		init_var[0] = x;
		init_var[1] = y;
		init_var[2] = z;
		init_var[3] = deg_x;// *rho_quat;
		init_var[4] = deg_y;// *rho_quat;
		init_var[5] = deg_z;// *rho_quat;
	};

	void solve() {
		try
		{
			/**
			* Create and initialize the constraints object
			*
			* First create it with default constraints (double type, min
			* -1.0e6, max 1.0e6) then set the first two elements to be of
			*  type real with x between -10, 10 and y between -100, 100.
			*/
			constraints_ptr constraints(boost::make_shared< constraints >(VARS_COUNT, -1.0e6, 1.0e6));
			(*constraints)[0] = boost::make_shared< int_constraint >(init_var[0] - 50, init_var[0] + 50);
			(*constraints)[1] = boost::make_shared< int_constraint >(init_var[1] - 50, init_var[1] + 50);
			(*constraints)[2] = boost::make_shared< int_constraint >(init_var[2] - 20, init_var[2] + 20);
			(*constraints)[3] = boost::make_shared< int_constraint >(init_var[3] - 3, init_var[3] + 3);
			(*constraints)[4] = boost::make_shared< int_constraint >(init_var[4] - 3, init_var[4] + 3);
			(*constraints)[5] = boost::make_shared< int_constraint >(init_var[5] - 3, init_var[5] + 3);


			/**
			* Instantiate the objective function
			*
			* The objective function can be any function or functor that
			* takes a de::DVectorPtr as argument and returns a double. It
			* can be passed as a reference, pointer or shared pointer.
			*/
			objective_function_ptr ofArray[THREAD_NUM];
			for (int ii = 0; ii < THREAD_NUM; ii++) {
				ofArray[ii] =boost::make_shared< DE_factor >();
			}
			

			/**
			* Instantiate two null listeners, one for the differential
			* evolution, the other one for the processors
			*/
			listener_ptr listener(boost::make_shared< null_listener >());
			processor_listener_ptr processor_listener(boost::make_shared< null_processor_listener >());

			/**
			* Instantiate the collection of processors with the number of
			* parallel processors (4), the objective function and the
			* listener
			*/
			
			processors< objective_function_ptr>::processors_ptr _processors(boost::make_shared< processors< objective_function_ptr> >(THREAD_NUM, ofArray, processor_listener));

			/**
			* Instantiate a simple termination strategy wich will stop the
			* optimization process after 10000 generations
			*/


			termination_strategy_ptr terminationStrategy(boost::make_shared< my_termination_strategy >(100, THRESHOLD_FINAL));


			/**
			* Instantiate the selection strategy - we'll use the best of
			* parent/child strategy
			*/
			selection_strategy_ptr selectionStrategy(boost::make_shared< best_parent_child_selection_strategy >());

			/**
			* Instantiate the mutation strategy - we'll use the mutation
			* strategy 1 with the weight and crossover factors set to 0.5
			* and 0.9 respectively
			*/
			mutation_strategy_arguments mutation_arguments(0.8, 0.9);
			mutation_strategy_ptr mutationStrategy(boost::make_shared< mutation_strategy_3 >(VARS_COUNT, mutation_arguments));

			/**
			* Instantiate the differential evolution using the previously
			* defined constraints, processors, listener, and the various
			* strategies
			*/
			differential_evolution< objective_function_ptr > de(VARS_COUNT, POPULATION_SIZE, _processors, constraints, true, terminationStrategy, selectionStrategy, mutationStrategy, listener);

			/**
			* Run the optimization process
			*/
			de.run();

			/**
			* Get the best individual resulted from the optimization
			* process
			*/
			best = de.best();

			/**
			* Print out the result
			*/
			std::cout << "minimum value for the " << ofArray[0]->name() << " is " << best->cost() << " for x=" << (*best->vars())[0] << ", y=" << (*best->vars())[1];
		}
		catch (const std::exception& e)
		{
			/**
			* Print out any errors that happened during the initialization
			* or optimization phases
			*/
			std::cout << "an error occurred: " << e.what();
		}
	}
};

class DE_Offline_Solver {
private:
	boost::shared_array<int> init_var;
	boost::shared_array<int> buffer_precision;
	boost::shared_array<int> buffer_l_boundary;
	boost::shared_array<int> buffer_r_boundary;
	boost::shared_array<Mat> model_DT_imgs;
	boost::shared_array<int> buffer_num;
	boost::shared_array<int> init_buffer_count_for_levels;

public:
	individual_ptr best;
	
	DE_Offline_Solver(boost::shared_array<int>& init_buffer_var, boost::shared_array<int>& init_buffer_precision,boost::shared_array<int>& init_buffer_num, boost::shared_array<int>& init_buffer_l_boundary, boost::shared_array<int>& init_buffer_r_boundary, boost::shared_array<Mat> & model_DT_imgs_, boost::shared_array<int> init_buffer_count_for_levels_) {
		init_var = init_buffer_var;
		buffer_precision = init_buffer_precision;
		buffer_l_boundary = init_buffer_l_boundary;
		buffer_r_boundary = init_buffer_r_boundary;
		model_DT_imgs = model_DT_imgs_;
		buffer_num = init_buffer_num;
		init_buffer_count_for_levels = init_buffer_count_for_levels_;
		
	}

	void solve() {
		try
		{
			/**
			* Create and initialize the constraints object
			*
			* First create it with default constraints (double type, min
			* -1.0e6, max 1.0e6) then set the first two elements to be of
			*  type real with x between -10, 10 and y between -100, 100.
			*/
			std::vector<int> vars_valide; std::vector<int> vars_non_valide;
			for (int i = 0; i < VARS_COUNT;i++) {
				if (buffer_num[i] > 1) vars_valide.push_back(i);
				else vars_non_valide.push_back(i);
			}
			constraints_ptr constraints(boost::make_shared< constraints >(vars_valide.size(), -1.0e6, 1.0e6));
			for (int i = 0; i < vars_valide.size(); i++) {
				(*constraints)[i] = boost::make_shared< int_constraint >(-(buffer_num[vars_valide[i]] - 1) / 2, +(buffer_num[vars_valide[i]] - 1) / 2);
			}
			


			/**
			* Instantiate the objective function
			*
			* The objective function can be any function or functor that
			* takes a de::DVectorPtr as argument and returns a double. It
			* can be passed as a reference, pointer or shared pointer.
			*/
			objective_function_ptr ofArray[THREAD_NUM];
			for (int ii = 0; ii < THREAD_NUM; ii++) {
				ofArray[ii] = boost::make_shared< DE_factor_offline >(vars_valide, vars_non_valide, model_DT_imgs, init_var, buffer_precision, buffer_l_boundary, buffer_r_boundary, init_buffer_count_for_levels);
			}


			/**
			* Instantiate two null listeners, one for the differential
			* evolution, the other one for the processors
			*/
			listener_ptr listener(boost::make_shared< null_listener >());
			processor_listener_ptr processor_listener(boost::make_shared< null_processor_listener >());

			/**
			* Instantiate the collection of processors with the number of
			* parallel processors (4), the objective function and the
			* listener
			*/

			processors< objective_function_ptr>::processors_ptr _processors(boost::make_shared< processors< objective_function_ptr> >(THREAD_NUM, ofArray, processor_listener));

			/**
			* Instantiate a simple termination strategy wich will stop the
			* optimization process after 10000 generations
			*/


			termination_strategy_ptr terminationStrategy(boost::make_shared< my_termination_strategy >(100, THRESHOLD_FINAL));


			/**
			* Instantiate the selection strategy - we'll use the best of
			* parent/child strategy
			*/
			selection_strategy_ptr selectionStrategy(boost::make_shared< best_parent_child_selection_strategy >());

			/**
			* Instantiate the mutation strategy - we'll use the mutation
			* strategy 1 with the weight and crossover factors set to 0.5
			* and 0.9 respectively
			*/
			mutation_strategy_arguments mutation_arguments(0.8, 0.9);
			mutation_strategy_ptr mutationStrategy(boost::make_shared< mutation_strategy_3 >(vars_valide.size(), mutation_arguments));

			/**
			* Instantiate the differential evolution using the previously
			* defined constraints, processors, listener, and the various
			* strategies
			*/
			differential_evolution< objective_function_ptr > de(vars_valide.size(), POPULATION_SIZE, _processors, constraints, true, terminationStrategy, selectionStrategy, mutationStrategy, listener);

			/**
			* Run the optimization process
			*/
			de.run();

			/**
			* Get the best individual resulted from the optimization
			* process
			*/
			best = de.best();

			/**
			* Print out the result
			*/
			std::cout << "minimum value for the " << ofArray[0]->name() << " is " << best->cost() << endl; //<< " for x=" << (*best->vars())[0] << ", y=" << (*best->vars())[1];
		}
		catch (const std::exception& e)
		{
			/**
			* Print out any errors that happened during the initialization
			* or optimization phases
			*/
			std::cout << "an error occurred: " << e.what();
		}
	}
};
class DE_Offline_Solver_modelCanny_camDT {
private:
	boost::shared_array<int> init_var;
	boost::shared_array<int> buffer_precision;
	boost::shared_array<int> buffer_l_boundary;
	boost::shared_array<int> buffer_r_boundary;
	boost::shared_array<std::vector<Point2i>> model_points_vec_array;

	boost::shared_array<int> buffer_num;
	boost::shared_array<int> init_buffer_count_for_levels;
	boost::shared_array<double> cache_match;

public:
	individual_ptr best;

	DE_Offline_Solver_modelCanny_camDT(boost::shared_array<int>& init_buffer_var, boost::shared_array<int>& init_buffer_precision, boost::shared_array<int>& init_buffer_num, boost::shared_array<int>& init_buffer_l_boundary, boost::shared_array<int>& init_buffer_r_boundary, boost::shared_array<std::vector<Point2i>> & model_points_vec_array_, boost::shared_array<int> init_buffer_count_for_levels_) {
		init_var = init_buffer_var;
		buffer_precision = init_buffer_precision;
		buffer_l_boundary = init_buffer_l_boundary;
		buffer_r_boundary = init_buffer_r_boundary;
		model_points_vec_array = model_points_vec_array_;
		buffer_num = init_buffer_num;
		init_buffer_count_for_levels = init_buffer_count_for_levels_;
		double* ptr = new double[init_buffer_count_for_levels_[0]];
		std::fill_n(ptr, init_buffer_count_for_levels_[0], -1.0);
		boost::shared_array<double> cache_match_(ptr);
		cache_match = cache_match_;
	}

	void solve() {
		try
		{
			/**
			* Create and initialize the constraints object
			*
			* First create it with default constraints (double type, min
			* -1.0e6, max 1.0e6) then set the first two elements to be of
			*  type real with x between -10, 10 and y between -100, 100.
			*/
			std::vector<int> vars_valide; std::vector<int> vars_non_valide;
			for (int i = 0; i < VARS_COUNT; i++) {
				if (buffer_num[i] > 1) vars_valide.push_back(i);
				else vars_non_valide.push_back(i);
			}
			constraints_ptr constraints(boost::make_shared< constraints >(vars_valide.size(), -1.0e6, 1.0e6));
			for (int i = 0; i < vars_valide.size(); i++) {
				(*constraints)[i] = boost::make_shared< int_constraint >(-(buffer_num[vars_valide[i]] - 1) / 2, +(buffer_num[vars_valide[i]] - 1) / 2);
			}



			/**
			* Instantiate the objective function
			*
			* The objective function can be any function or functor that
			* takes a de::DVectorPtr as argument and returns a double. It
			* can be passed as a reference, pointer or shared pointer.
			*/
			objective_function_ptr ofArray[THREAD_NUM];
			for (int ii = 0; ii < THREAD_NUM; ii++) {
				ofArray[ii] = boost::make_shared< DE_factor_offline_modelCanny_camDT >(vars_valide, vars_non_valide, model_points_vec_array, init_var, buffer_precision, buffer_l_boundary, buffer_r_boundary, init_buffer_count_for_levels,cache_match);
			}


			/**
			* Instantiate two null listeners, one for the differential
			* evolution, the other one for the processors
			*/
			listener_ptr listener(boost::make_shared< null_listener >());
			processor_listener_ptr processor_listener(boost::make_shared< null_processor_listener >());

			/**
			* Instantiate the collection of processors with the number of
			* parallel processors (4), the objective function and the
			* listener
			*/

			processors< objective_function_ptr>::processors_ptr _processors(boost::make_shared< processors< objective_function_ptr> >(THREAD_NUM, ofArray, processor_listener));

			/**
			* Instantiate a simple termination strategy wich will stop the
			* optimization process after 10000 generations
			*/


			termination_strategy_ptr terminationStrategy(boost::make_shared< my_termination_strategy >(100, THRESHOLD_FINAL));


			/**
			* Instantiate the selection strategy - we'll use the best of
			* parent/child strategy
			*/
			selection_strategy_ptr selectionStrategy(boost::make_shared< best_parent_child_selection_strategy >());

			/**
			* Instantiate the mutation strategy - we'll use the mutation
			* strategy 1 with the weight and crossover factors set to 0.5
			* and 0.9 respectively
			*/
			mutation_strategy_arguments mutation_arguments(0.8, 0.9);
			mutation_strategy_ptr mutationStrategy(boost::make_shared< mutation_strategy_3 >(vars_valide.size(), mutation_arguments));

			/**
			* Instantiate the differential evolution using the previously
			* defined constraints, processors, listener, and the various
			* strategies
			*/
			differential_evolution< objective_function_ptr > de(vars_valide.size(), POPULATION_SIZE, _processors, constraints, true, terminationStrategy, selectionStrategy, mutationStrategy, listener);

			/**
			* Run the optimization process
			*/
			de.run();

			/**
			* Get the best individual resulted from the optimization
			* process
			*/
			best = de.best();

			/**
			* Print out the result
			*/
			std::cout << "minimum value for the " << ofArray[0]->name() << " is " << best->cost() << endl; //<< " for x=" << (*best->vars())[0] << ", y=" << (*best->vars())[1];
		}
		catch (const std::exception& e)
		{
			/**
			* Print out any errors that happened during the initialization
			* or optimization phases
			*/
			std::cout << "an error occurred: " << e.what();
		}
	}
};
#endif
