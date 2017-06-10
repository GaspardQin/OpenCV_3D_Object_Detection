// creat_DLL.cpp : 定义 DLL 应用程序的导出函数。
//
#include "stdafx.h"
#include "creat_DLL.h"

namespace Object_Detection {
	boost::mutex gl_mutex;
	boost::shared_mutex cv_cache_mutex;
	HANDLE sentModelEvent;
	HANDLE nextModelEvent;//全局变量应在相应cpp下先声明，再在thread_variables.h中声明extern
	HANDLE readImgEvent;
	boost::atomic_int iteral_count(0);
	int option;
	int print_option = 0;
	bool print_detail = true;
	//camera options
	int WINDOW_WIDTH = 2592;
	int WINDOW_HEIGHT = 1944; //pixel
	int ROI_WIDTH = 1000;
	int ROI_HEIGHT = 1200;  //pixel
	double FOCAL_DISTANCE = 8.0; //mm
	double CCD_WIDTH = 5.709; //mm
	bool is_create_sample = false;
	//DE_options
	int THREAD_NUM = 4;
	int POPULATION_SIZE = 30;
	int THRESHOLD_FINAL = 1000;
	int GEN_MAX = 50;
	double DE_wight_factor = 0.4;
	double DE_crossover_factor = 0.6;
	double DT_match_min_factor = 0;
	double DT_match_max_factor = 0.6;

	int ObjectDetection::run() {
		try {


			sentModelEvent = CreateEvent(NULL, false, false, (LPTSTR)"sendingModelEvent");
			nextModelEvent = CreateEvent(NULL, false, false, (LPTSTR)"renderingNextModelEvent");
			readImgEvent = CreateEvent(NULL, false, false, (LPTSTR)"readingModelEvent");
			ResetEvent(sentModelEvent);
			ResetEvent(readImgEvent);
			ResetEvent(nextModelEvent);

			if ((!sentModelEvent) || (!nextModelEvent) || (!readImgEvent))
			{
				std::cout << "Failed to CreateEvent !" << std::endl;
				return 0;
			}
			
			boost::thread_group tg;
			tg.add_thread(new boost::thread(boost::bind(&cvThread::cvModelThreadFun, cv_thread)));
			tg.add_thread(new boost::thread(boost::bind(&glThread::glThreadFun, gl_thread)));
			//boost::thread h_cv_thread(boost::bind(&cvThread::cvModelThreadFun, cv_thread));
			//boost::thread h_gl_thread(boost::bind(&glThread::glThreadFun, gl_thread));
			//h_cv_thread.join();
			//h_gl_thread.join();
			tg.join_all();

		}
		catch (std::invalid_argument &error) {
			std::cerr << " Invalid_argument " << error.what() << std::endl;
		}
		return 0;
	}
	void ObjectDetection::setEdgeCompareMethod(int option_) {
		try {

			option = option_;
			if (option != MODEL_CANNY_CAM_DT_ONLINE && option != MODEL_CANNY_CAM_DT_OFFLINE && option != MODEL_DT_CAM_CANNY_ONLINE && option != MODEL_DT_CAM_CANNY_ONLINE_ROI) {
				throw std::invalid_argument("Input EdgeCompareMethod is not valide");
			}
		}
		catch (std::invalid_argument &error) {
			std::cerr << " Invalid_argument " << error.what() << std::endl;
		}
	}
	void ObjectDetection::setInitialValue(int x, int y, int z, int x_deg, int y_deg, int z_deg) {//相机坐标系
		discrete_info.setInitValue(x, y, z, x_deg, y_deg, z_deg);

	
	}
	void ObjectDetection::setDiscretePrecision(int x, int y, int z, int x_deg, int y_deg, int z_deg) {
		discrete_info.setPrecision(x, y, z, x_deg, y_deg, z_deg);

	}
	void ObjectDetection::setDiscreteBoundary(int x, int y, int z, int x_deg, int y_deg, int z_deg) {//偏离初始值的范围
		discrete_info.setBoundary(x, y, z, x_deg, y_deg, z_deg);
	}
	void ObjectDetection::createSampleImg(int x, int y, int z, int x_deg, int y_deg, int z_deg)
	{
		//开启openGL线程
		sentModelEvent = CreateEvent(NULL, false, false, (LPTSTR)"sendingModelEvent");
		nextModelEvent = CreateEvent(NULL, false, false, (LPTSTR)"renderingNextModelEvent");
		readImgEvent = CreateEvent(NULL, false, false, (LPTSTR)"readingModelEvent");
		ResetEvent(sentModelEvent);
		ResetEvent(readImgEvent);
		ResetEvent(nextModelEvent);
		readSrcImg = cv::Mat::zeros(WINDOW_HEIGHT, WINDOW_WIDTH, CV_8UC1); //CV_8UC3);//the raw img got from the buffer of OpenGL;
		readSrcImgROI = cv::Mat::zeros(ROI_HEIGHT, ROI_WIDTH, CV_8UC1); //CV_8UC3);//the raw img of ROI got from the buffer of OpenGL;

		if ((!sentModelEvent) || (!nextModelEvent) || (!readImgEvent))
		{
			std::cout << "Failed to CreateEvent !" << std::endl;
			return ;
		}

		boost::thread_group tg;
		tg.add_thread(new boost::thread(boost::bind(&glThread::glThreadFun, gl_thread)));


		is_create_sample = true;
		ResetEvent(sentModelEvent);
		SetEvent(readImgEvent);
		
		rotate_degree_set[0] = x_deg;
		rotate_degree_set[1] = y_deg;
		rotate_degree_set[2] = z_deg;
		pos_model_set[0] = x;
		pos_model_set[1] = y;
		pos_model_set[2] = z;
		//quat_set = glm::quat(glm::vec3(glm::radians(rotate_degree_set[0]), glm::radians(rotate_degree_set[1]), glm::radians(rotate_degree_set[2])));
		SetEvent(nextModelEvent);
		
		WaitForSingleObject(sentModelEvent, INFINITE);
		namedWindow("creatSample", WINDOW_NORMAL);
		resizeWindow("creatSample", WINDOW_WIDTH , WINDOW_HEIGHT );
		imshow("creatSample", readSrcImg);
		waitKey();
		imwrite("../model/sample.jpg", readSrcImg);
		imwrite("../model/sample.bmp", readSrcImg);
		SetEvent(readImgEvent);
		SetEvent(nextModelEvent);
		is_create_sample = false;
		return;
	}
}