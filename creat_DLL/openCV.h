#pragma once

#include "detectionMethodWithoutBuffer.h"
namespace Object_Detection {
	class cvThread
	{
	public:
		void cvModelThreadFun();
		void creatSample();
		void cvThread::test_camera();
		void setCamImgPath(string path_) { cam_img_path = path_; };
		void setCamImgSrcOption(int option_) { cam_src_option = option_; };

	private:
		string cam_img_path;
		int cam_src_option;
	};
	
}