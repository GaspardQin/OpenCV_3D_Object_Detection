#pragma once

#ifndef _DE_H_
#define _DE_H_
#include "stdafx.h"
#include "MatchEdges.h"



#define VARS_COUNT 6


	/**
	* Objective function to optimize is "sumDT"
	*/

namespace Object_Detection {
	using namespace de;
	using namespace Object_Detection;
	class DE_Factor :public objective_function, public MatchEdges
	{
	private:
		int params[6];
		std::vector<int> vars_valide; std::vector<int> vars_non_valide;

		double calculateDTfactor(
			const int* params_array
		)const;



		const void drawPoints(Mat &img, std::vector<Point2f> points, const Scalar& color)const;
		const void debugShowMatch(const int* var)const;

	public:
		DE_Factor(std::vector<int>& vars_valide_, std::vector<int>& vars_non_valide_) :objective_function("sumDT"), MatchEdges()
		{
			vars_valide = vars_valide_;
			vars_non_valide = vars_non_valide_;
		}

		double operator()(de::DVectorPtr args);
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
		bool event(individual_ptr best, size_t genCount);
	};

	class DE_Solver {
	private:
	public:
		individual_ptr best;
		void solve();
	};
}
#endif
