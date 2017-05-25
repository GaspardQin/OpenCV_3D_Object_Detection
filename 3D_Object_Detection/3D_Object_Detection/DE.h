#pragma once

#ifndef _DE_H_
#define _DE_H_
#include "differential_evolution.hpp"
#include "MatchSolver.h"
#include "objective_function.h"
#include "thread_variables.h"
using namespace de;
#define THREAD_NUM 4
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
		Mat cam_src = imread("./model/sample.jpg", CV_8UC1);
		Mat cam_canny_img;
		Canny(cam_src, cam_canny_img, 50, 200);
		cost_factor_ptr = boost::make_shared<CostFactorPyramid>(cam_canny_img,0);
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
		return cost_factor_ptr->calculateDTfactorPyramid(params);
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
		return (genCount < maxGen) && (best->cost() > threshold);
	}
};
#define VARS_COUNT 6
#define POPULATION_SIZE 15
class DEsolver {
private:
	double init_var[6];
public:
	individual_ptr best;
	DEsolver(double * pos_estimated, double * quat_estimated) {
		init_var[0] = pos_estimated[0];
		init_var[1] = pos_estimated[1];
		init_var[2] = pos_estimated[2];
		init_var[3] = quat_estimated[0]*rho_quat;
		init_var[4] = quat_estimated[1]*rho_quat;
		init_var[5] = quat_estimated[2]*rho_quat;
	}
	void setInitVar(double x, double y, double z, double quat_x, double quat_y, double quat_z) {
		init_var[0] = x;
		init_var[1] = y;
		init_var[2] = z;
		init_var[3] = quat_x*rho_quat;
		init_var[4] = quat_y*rho_quat;
		init_var[5] = quat_z*rho_quat;
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
			(*constraints)[0] = boost::make_shared< real_constraint >(init_var[0] - 10, init_var[0] + 10);
			(*constraints)[1] = boost::make_shared< real_constraint >(init_var[1] - 10, init_var[1] + 10);
			(*constraints)[2] = boost::make_shared< real_constraint >(init_var[2] - 10, init_var[2] + 10);
			(*constraints)[3] = boost::make_shared< real_constraint >(std::max(init_var[3] - 20, -0.5*rho_quat), std::min(init_var[3] + 20, 0.5*rho_quat));
			(*constraints)[4] = boost::make_shared< real_constraint >(std::max(init_var[4] - 20, -0.5*rho_quat), std::min(init_var[4] + 20, 0.5*rho_quat));
			(*constraints)[5] = boost::make_shared< real_constraint >(std::max(init_var[5] - 20, -0.5*rho_quat), std::min(init_var[5] + 20, 0.5*rho_quat));


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


			termination_strategy_ptr terminationStrategy(boost::make_shared< my_termination_strategy >(100,150));


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
			mutation_strategy_arguments mutation_arguments(0.7, 0.6);
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


#endif
