#pragma once
#ifndef _creat_DLL_H_
#define _creat_DLL_H_
#ifdef CREAT_DLL_EXPORTS
#define OBJECTDETECTION_API __declspec(dllexport) 
#else
#define OBJECTDETECTION_API __declspec(dllimport) 
#endif
#include "openCV.h"
#include "loadModel.h"

namespace Object_Detection {
	using namespace std;	
	class ObjectDetection
	{
	public:
		int OBJECTDETECTION_API run();
		void OBJECTDETECTION_API setEdgeCompareMethod(int option_);// can only be MODEL_CANNY_CAM_DT_ONLINE,MODEL_CANNY_CAM_DT_OFFLINE,MODEL_DT_CAM_CANNY_ONLINE,or MODEL_DT_CAM_CANNY_ONLINE_ROI
		void OBJECTDETECTION_API setDiskImgPath(string& path_) {
			cv_thread.setCamImgPath(path_);
		};
		void OBJECTDETECTION_API setImgSourceOption(int option_) {
			cv_thread.setCamImgSrcOption(option_);
		};
		void OBJECTDETECTION_API setWindowSize(int width, int height) {
			//pixel
			WINDOW_WIDTH = width;
			WINDOW_HEIGHT = height;
		};
		void OBJECTDETECTION_API setROISize(int ROI_width, int ROI_height) {
			//pixel
			//This option can only be used when MODEL_DT_CAM_CANNY_ONLINE_ROI is set
			ROI_WIDTH = ROI_width;
			ROI_HEIGHT = ROI_height;
		}
		void OBJECTDETECTION_API setCCDWidth(double ccd_width) {
			//mm
			CCD_WIDTH = ccd_width;
		}
		void OBJECTDETECTION_API setFocalDistance(double focal_distance) {
			//mm
			FOCAL_DISTANCE = focal_distance;
		}
		void OBJECTDETECTION_API setDEoptions(int thread_num_ = 4, int population_size_ = 30, int threshold_final_ = 1000,int max_generation_ = 50, double DE_wight_factor_ = 0.4, double DE_crossover_factor_ = 0.6) {
			THREAD_NUM = thread_num_;
			POPULATION_SIZE = population_size_;
			THRESHOLD_FINAL = threshold_final_;
			GEN_MAX = max_generation_;
			DE_wight_factor = DE_wight_factor_;
			DE_crossover_factor = DE_crossover_factor_;
		}
		void OBJECTDETECTION_API setDToptions(double DT_match_min_factor_ = 0.0, double DT_match_max_factor_ = 0.6) {
			DT_match_min_factor = DT_match_min_factor_;
			DT_match_max_factor = DT_match_max_factor_;
		}
		void OBJECTDETECTION_API setPrintOptions(int option_) {
			//can be NON_PRINT, MIN_PRINT or DETAIL_PRINT
			try {


				if (option_ != NON_PRINT && option_ != MIN_PRINT && option_ != DETAIL_PRINT) {
					throw std::invalid_argument("Input PrintOptions is not valide");
				}
				print_option = option_;
			}
			catch (std::invalid_argument &error) {
				std::cerr << " Invalid_argument " << error.what() << std::endl;
			}
		};
		void OBJECTDETECTION_API setInitialValue(int x, int y, int z, int x_deg, int y_deg, int z_deg);//相机坐标系
		void OBJECTDETECTION_API setDiscretePrecision(int x, int y, int z, int x_deg, int y_deg, int z_deg);
		void OBJECTDETECTION_API setDiscreteBoundary(int x, int y, int z, int x_deg, int y_deg, int z_deg);//偏离初始值的范围
		void OBJECTDETECTION_API createSampleImg(int x, int y, int z, int x_deg, int y_deg, int z_deg);
	private:
		cvThread cv_thread;
		glThread gl_thread;
	};







}

#endif