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
#define THRESHOLD_FINAL 1000
/**
* Objective function to optimize is "sumDT" 
*/
class DE_factor :public objective_function
{
private:
	boost::shared_ptr<CostFactorOnline> cost_factor_ptr;
	int params[6];
	std::vector<int> vars_valide; std::vector<int> vars_non_valide;
public:
	DE_factor(std::vector<int>& vars_valide_, std::vector<int>& vars_non_valide_) :objective_function("sumDT")
	{
		cost_factor_ptr = boost::make_shared<CostFactorOnline>();
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
			params[vars_valide[i]] = (*args)[vars_valide[i]] *discrete_info.precision[vars_valide[i]] + discrete_info.init_var[vars_valide[i]];
		}
		for (int i = 0; i < vars_non_valide.size(); i++) {
			params[vars_non_valide[i]] = discrete_info.init_var[vars_non_valide[i]];
		}

		return cost_factor_ptr->calculateDTfactorPyramid(params);
	}
};
class DE_OnlineFactor:public objective_function
{
private:
	boost::shared_ptr<CostFactorOnline> cost_factor_ptr;
	int params[6];
	std::vector<int> vars_valide; std::vector<int> vars_non_valide;
public:
	DE_OnlineFactor(std::vector<int>& vars_valide_, std::vector<int>& vars_non_valide_) :objective_function("sumDT")
	{
		cost_factor_ptr = boost::make_shared<CostFactorOnline>();
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
			params[vars_valide[i]] = (*args)[vars_valide[i]] * discrete_info.precision[vars_valide[i]] + discrete_info.init_var[vars_valide[i]];
		}
		for (int i = 0; i < vars_non_valide.size(); i++) {
			params[vars_non_valide[i]] = discrete_info.init_var[vars_non_valide[i]];
		}

		return cost_factor_ptr->calculateDTfactorPyramid(params);
	}
};

class DE_factor_offline_modelCanny_camDT :public objective_function
{
private:
	boost::shared_ptr<CostFactorDT_Offline_modelCanny_camDT> cost_factor_ptr;
	int params[6];
	std::vector<int> vars_valide; std::vector<int> vars_non_valide;
public:
	DE_factor_offline_modelCanny_camDT(std::vector<int>& vars_valide_, std::vector<int>& vars_non_valide_) :objective_function("sumDT_offline")
	{

		cost_factor_ptr = boost::make_shared<CostFactorDT_Offline_modelCanny_camDT>();
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
			params[vars_valide[i]] = (*args)[vars_valide[i]] * discrete_info.precision[vars_valide[i]] + discrete_info.init_var[vars_valide[i]];
		}
		for (int i = 0; i < vars_non_valide.size(); i++) {
			params[vars_non_valide[i]] = discrete_info.init_var[vars_non_valide[i]];
		}

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
		std::cout << "genCount : " << genCount << std::endl;
		return (genCount < maxGen) && (best->cost() > threshold);
	}
};

class DE_OnlineSolver {
public:
	individual_ptr best;
	DE_OnlineSolver( ) {
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
				if (discrete_info.num[i] > 1) vars_valide.push_back(i);
				else vars_non_valide.push_back(i);
			}
			constraints_ptr constraints(boost::make_shared< constraints >(vars_valide.size(), -1.0e6, 1.0e6));
			for (int i = 0; i < vars_valide.size(); i++) {
				(*constraints)[i] = boost::make_shared< int_constraint >(-(discrete_info.num[vars_valide[i]] - 1) / 2, +(discrete_info.num[vars_valide[i]] - 1) / 2);
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
				ofArray[ii] = boost::make_shared< DE_OnlineFactor >(vars_valide, vars_non_valide);
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
			mutation_strategy_arguments mutation_arguments(0.4, 0.6);
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

public:
	individual_ptr best;

	DE_Offline_Solver_modelCanny_camDT() {

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
				if (discrete_info.num[i] > 1) vars_valide.push_back(i);
				else vars_non_valide.push_back(i);
			}
			constraints_ptr constraints(boost::make_shared< constraints >(vars_valide.size(), -1.0e6, 1.0e6));
			for (int i = 0; i < vars_valide.size(); i++) {
				(*constraints)[i] = boost::make_shared< int_constraint >(-(discrete_info.num[vars_valide[i]] - 1) / 2, +(discrete_info.num[vars_valide[i]] - 1) / 2);
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
				ofArray[ii] = boost::make_shared< DE_factor_offline_modelCanny_camDT >(vars_valide, vars_non_valide);
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
