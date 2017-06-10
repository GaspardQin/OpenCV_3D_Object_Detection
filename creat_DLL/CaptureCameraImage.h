#pragma once
//Using DaHeng SDK
#include <iostream>



namespace Object_Detection {
	using namespace std;
	class DaHengCamera {
	public:

		//�û��̳е����¼�������
		class MyDeviceOfflineEventHandler : public IDeviceOfflineEventHandler
		{
		public:
			void DoOnDeviceOfflineEvent(void* pUserParam)
			{
				cout << "�յ��豸�����¼�!" << endl;
			}
		};

		//�û��̳����Ը����¼�������
		class MyFeatureEventHandler : public IFeatureEventHandler
		{
		public:
			void DoOnFeatureEvent(const GxIAPICPP::gxstring&strFeatureName, void* pUserParam)
			{
				cout << "�յ��ع�����¼�!" << endl;
			}
		};
		//�û��̳вɼ��¼�������

		class MyCaptureEventHandler : public ICaptureEventHandler
		{
		public:
			void DoOnImageCaptured(CImageDataPointer&objImageDataPointer, void* pUserParam)
			{
				cout << "�յ�һ֡ͼ��!" << endl;
				cout << "ImageInfo: " << objImageDataPointer->GetStatus() << endl;
				cout << "ImageInfo: " << objImageDataPointer->GetWidth() << endl;
				cout << "ImageInfo: " << objImageDataPointer->GetHeight() << endl;
				cout << "ImageInfo: " << objImageDataPointer->GetPayloadSize() << endl;
			}
		};



		int captureCamera();


	};
}