#pragma once
//Using DaHeng SDK
#include <iostream>



namespace Object_Detection {
	using namespace std;
	class DaHengCamera {
	public:

		//用户继承掉线事件处理类
		class MyDeviceOfflineEventHandler : public IDeviceOfflineEventHandler
		{
		public:
			void DoOnDeviceOfflineEvent(void* pUserParam)
			{
				cout << "收到设备掉线事件!" << endl;
			}
		};

		//用户继承属性更新事件处理类
		class MyFeatureEventHandler : public IFeatureEventHandler
		{
		public:
			void DoOnFeatureEvent(const GxIAPICPP::gxstring&strFeatureName, void* pUserParam)
			{
				cout << "收到曝光结束事件!" << endl;
			}
		};
		//用户继承采集事件处理类

		class MyCaptureEventHandler : public ICaptureEventHandler
		{
		public:
			void DoOnImageCaptured(CImageDataPointer&objImageDataPointer, void* pUserParam)
			{
				cout << "收到一帧图像!" << endl;
				cout << "ImageInfo: " << objImageDataPointer->GetStatus() << endl;
				cout << "ImageInfo: " << objImageDataPointer->GetWidth() << endl;
				cout << "ImageInfo: " << objImageDataPointer->GetHeight() << endl;
				cout << "ImageInfo: " << objImageDataPointer->GetPayloadSize() << endl;
			}
		};



		int captureCamera();


	};
}