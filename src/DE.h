#pragma once

#ifndef _DE_H_
#define _DE_H_
#include "differential_evolution.hpp"
#include "MatchEdges.h"
#include "objective_function.h"
#include "thread_variables.h"
using namespace de;
#define THREAD_NUM 1
#define VARS_COUNT 6
#define VARS_THRESHOLD 1
#define SAME_GEN_MAX 200000
#define POPULATION_SIZE 30
#define THRESHOLD_FINAL 0.00000001
#define GEN_MAX 30

/**
* Objective function to optimize is "sumDT" 
*/


class DE_Factor :public objective_function, public MatchEdges
{
private:
	boost::shared_ptr<CostFactor> cost_factor_ptr;
	int params[6]; double double_params[6];
	std::vector<int> vars_valide; std::vector<int> vars_non_valide;
	double calculateDTfactor_double(
		const double* params_array
	)const {

		double dist;
		switch (option)
		{
		//case MODEL_CANNY_CAM_DT_ONLINE:
		//	dist = MatchOnline_modelCannycamDT(params_array, 0.3, 0.8); break;
		//case MODEL_CANNY_CAM_DT_OFFLINE:
		//	dist = MatchOffline_modelCannycamDT(params_array, 0.3, 0.8); break;
		case MODEL_DT_CAM_CANNY_ONLINE:
			dist = MatchOnline_modelDTcamCanny_continuous(params_array, 0.0, 0.1); break;
		case MODEL_DT_CAM_CANNY_ONLINE_ROI:
			dist = MatchOnline_modelDTcamCannyROI_continuous(params_array, 0.0, 0.1); break;
		default:
			cout << "option input is not valide" << endl;
			break;
		}

		cout << "params input: x: " << params_array[0] << " y: " << params_array[1] << " z: " << params_array[2] << " x_deg: " << params_array[3] << " y_deg: " << params_array[4] << " z_deg: " << params_array[5] << endl;
		cout << "DT score iteral " << dist << endl;
		//	debugShowMatch(params_array);
		//	waitKey(10);
		iteral_count = iteral_count + 1;
		return dist;
	}
	double calculateDTfactor(
		const int* params_array
	)const {

		double dist;
			switch (option)
			{
			case MODEL_CANNY_CAM_DT_ONLINE: 
				dist = MatchOnline_modelCannycamDT(params_array, 0.3, 0.8); break;
			case MODEL_CANNY_CAM_DT_OFFLINE: 
				dist = MatchOffline_modelCannycamDT(params_array, 0.3, 0.8); break;
			case MODEL_DT_CAM_CANNY_ONLINE: 
				dist = MatchOnline_modelDTcamCanny(params_array, 0.3, 0.8); break;
			case MODEL_DT_CAM_CANNY_ONLINE_ROI:
				dist = MatchOnline_modelDTcamCannyROI(params_array, 0.3, 0.8); break;
			default:
				cout << "option input is not valide" << endl;
				break;
			}

		cout << "params input: x: " << params_array[0] << " y: " << params_array[1] << " z: " << params_array[2] << " x_deg: " << params_array[3] << " y_deg: " << params_array[4] << " z_deg: " << params_array[5] << endl;
		cout << "DT score iteral " << dist << endl;
		//	debugShowMatch(params_array);
		//	waitKey(10);
		iteral_count = iteral_count + 1;
		return dist;
	}



	const void drawPoints(Mat &img, std::vector<Point2f> points, const Scalar& color)const {
		int i;
		for (i = 0; i < points.size(); i++) {
			circle(img, points[i], 5, color);
		}
	}
	const void debugShowMatch(const double* var)const {
		Mat back_ground = cam_img_color_src.clone();



		std::vector<Point2i> contours_points;
		getModelImgUchar(var);
		cv::findNonZero(readSrcImg, contours_points);
		ResetEvent(readImgEvent);
		SetEvent(readImgEvent);
		Vec3b * temp;
		for(std::vector<Point2i>::iterator i =contours_points.begin();i < contours_points.end();i++)
		{
			temp = &back_ground.at<Vec3b>(i->y,i->x);
			temp[0] = 200; //Blue;
			temp[1] = 100; //g;
			temp[2] = 0; //r;
		}
		pyrDown(back_ground, back_ground);
		imshow("debugShowMatchImgs", back_ground);
		waitKey(20);
	}

public:
	DE_Factor(std::vector<int>& vars_valide_, std::vector<int>& vars_non_valide_) :objective_function("sumDT"), MatchEdges()
	{
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
		if (discrete_option == DISCRETE_MATCH) {
			for (int i = 0; i < vars_valide.size(); i++) {
				params[vars_valide[i]] = (*args)[vars_valide[i]] * discrete_info.precision[vars_valide[i]] + discrete_info.init_var[vars_valide[i]];
			}
			for (int i = 0; i < vars_non_valide.size(); i++) {
				params[vars_non_valide[i]] = discrete_info.init_var[vars_non_valide[i]];
			}
			return calculateDTfactor(params);
		} 
		else {
			for (int i = 0; i < vars_valide.size(); i++) {
				double_params[vars_valide[i]] = (*args)[vars_valide[i]];
			}

			for (int i = 0; i < vars_non_valide.size(); i++) {
				double_params[vars_non_valide[i]] = continuous_info.init_var[vars_non_valide[i]];
			}
			namedWindow("debugShowMatchImgs", 1);
			debugShowMatch(double_params);
			return calculateDTfactor_double(double_params);
		}


		
	}
};

class my_termination_strategy :public de::termination_strategy {
private:
	double threshold;
	size_t maxGen;
	double vars_threshold;
	double old_vars[6];
	bool islike = true;
	int terminal_iteral_count = 0;
public:
	my_termination_strategy(size_t maxGen_, double threshold_,double vars_threshold_) {
		maxGen = maxGen_;
		threshold = threshold_;
		vars_threshold = vars_threshold_;
	}
	bool event(individual_ptr best, size_t genCount)
	{
		boost::lock_guard<boost::mutex> lock(terminal_mutex);
		for (int i = 0; i < vars_valide.size(); i++) {
			if (abs(old_vars[i] - (*best->vars())[i]) > vars_threshold){
				islike = false;
				break;
			}
		}
		if (islike == false) {
			for (int i = 0; i < vars_valide.size(); i++) {
				old_vars[i] = (*best->vars())[i];
			}
			terminal_iteral_count = 0;
		}
		else {
			terminal_iteral_count++;
		}

		std::cout << "genCount : " << genCount << std::endl;
		return (genCount < maxGen) && (best->cost() > threshold && (terminal_iteral_count < SAME_GEN_MAX));
	}
};

class DE_Solver {
private:
public:
	individual_ptr best;
	DE_Solver() {};

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
			constraints_ptr constraints_;
			if (discrete_option == DISCRETE_MATCH) {
				for (int i = 0; i < VARS_COUNT; i++) {
					if (discrete_info.num[i] > 1) vars_valide.push_back(i);
					else vars_non_valide.push_back(i);
				}
				constraints_ = boost::make_shared< constraints >(vars_valide.size(), -1.0e6, 1.0e6);
				for (int i = 0; i < vars_valide.size(); i++) {
					(*constraints_)[i] = boost::make_shared< int_constraint >(-(discrete_info.num[vars_valide[i]] - 1) / 2, +(discrete_info.num[vars_valide[i]] - 1) / 2);
				}
			}
			else {
				for (int i = 0; i < VARS_COUNT; i++) {
					if (continuous_info.r_boundary[i]- continuous_info.l_boundary[i] > 0) vars_valide.push_back(i);
					else vars_non_valide.push_back(i);
				}
				constraints_ = boost::make_shared< constraints >(vars_valide.size(), -1.0e6, 1.0e6);
				for (int i = 0; i < vars_valide.size(); i++) {
					(*constraints_)[i] = boost::make_shared< real_constraint >(continuous_info.l_boundary[i],continuous_info.r_boundary[i]);
				}
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
				ofArray[ii] = boost::make_shared< DE_Factor >(vars_valide, vars_non_valide);
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


			termination_strategy_ptr terminationStrategy(boost::make_shared< my_termination_strategy >(GEN_MAX, THRESHOLD_FINAL, VARS_THRESHOLD));


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
			mutation_strategy_arguments mutation_arguments(0.6, 0.7);
			mutation_strategy_ptr mutationStrategy(boost::make_shared< mutation_strategy_3 >(vars_valide.size(), mutation_arguments));

			/**
			* Instantiate the differential evolution using the previously
			* defined constraints, processors, listener, and the various
			* strategies
			*/
			differential_evolution< objective_function_ptr > de(vars_valide.size(), POPULATION_SIZE, _processors, constraints_, true, terminationStrategy, selectionStrategy, mutationStrategy, listener);

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
