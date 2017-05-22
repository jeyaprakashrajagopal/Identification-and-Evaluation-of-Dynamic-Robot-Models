/****************************************************************************************
 * Filename 	: ModelOptimizer.hpp
 * Path			: /models_evaluation/include/models_evaluation
 * Institute	: Hochschule Bonn-Rhein-Sieg
 * Author 		: Jeyaprakash Rajagopal
 * Advisors		: Prof. Dr. Paul G Plöger, Sven Schneider
 * Explanation 	: Header file for ModelOptimizer
 **************************************************************************************/
#ifndef __MODEL_OPTIMIZER_
#define __MODEL_OPTIMIZER_
#include <models_evaluation.hpp>
#include <fstream>
#include <stdlib.h>
#include <kdl/frames_io.hpp>
#include <kdl/kinfam_io.hpp>
#include <kdl/chaindynparam.hpp>
#include <nlopt.hpp>
#define NO_OF_PARAMS 12
#define NO_OF_PARAMS_DYN 16
/// Default gravity value
#define GRAV -9.81
using namespace nlopt;

class ModelOptimizer
{
	public:
		/**
		 * @brief Constructor declaration.
		*/
		ModelOptimizer();
		/// Struct declaration for the dynamic parameters that are going to be optimized
		struct DynamicParameters
		{
			double mass[4];
			double com_radius[4];
			double com_angle[4];
			double com_z[4];
		};
		/// Struct declaration for the static configurations that are going to be optimized
		struct StaticParameters
		{
			double mass[4];
			double com_radius[4];
			double com_angle[4];
		};
		void setDynamicParameters(DynamicParameters dynParam){dyn_p_ = dynParam;}
		void setStaticParameters(StaticParameters staParam){sta_p_ = staParam;}

		/**
		 * @brief Optimization for static joint effort.
		 * @param .
		 * @return void.
		*/
		void getOptimizedStaticParams();
		/**
		 * @brief Optimization for dynamics joint effort.
		 * @param .
		 * @return void.
		*/
		void getOptimizedDynamicParams();
		/// Structure to hold the experimental data from youBot manipulator
		struct State
		{
			double pos[5];
			double vel[5];
			double trq[5];
		};
		void loadParams(string s);
		DynamicParameters dyn_p_;
		StaticParameters sta_p_;
		/// To set the parameters
		void loadData(string s);
		/// To get the static joint effort computations done
		std::vector<double> getStaticEffort(std::vector<double> q,std::vector<double> qdot,std::vector<double> qddot);
		/// To get the dynamics joint effort computations done
		std::vector<double> getDynamicEffort(std::vector<double> q,std::vector<double> qdot,std::vector<double> qddot);
		/// To save the parameters into a file for both the static and dynamic configurations
		void saveParams();

};
#endif
