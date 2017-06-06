#pragma once
//Using DaHeng SDK
#include <iostream>
#include "thread_variables.h"
using namespace std;
//请用户提前配置好工程头文件目录,需要包含GalaxyIncludes.h

#include"GalaxyIncludes.h"



//用户继承掉线事件处理类
class MyDeviceOfflineEventHandler: public IDeviceOfflineEventHandler
{
public:
	void DoOnDeviceOfflineEvent(void* pUserParam)
	{
		cout << "收到设备掉线事件!" << endl;
	}
};

 //用户继承属性更新事件处理类
class MyFeatureEventHandler: public IFeatureEventHandler
{
public:
	void DoOnFeatureEvent(const GxIAPICPP::gxstring&strFeatureName, void* pUserParam)
	{
		cout << "收到曝光结束事件!" << endl;
	}
};
//用户继承采集事件处理类

class MyCaptureEventHandler: public ICaptureEventHandler
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



int captureCamera()
{

	//声明事件回调对象指针
	IDeviceOfflineEventHandler* pDeviceOfflineEventHandler = NULL;///<掉线事件回调对象
	IFeatureEventHandler* pFeatureEventHandler = NULL;///<远端设备事件回调对象
	ICaptureEventHandler* pCaptureEventHandler = NULL;///<采集回调对象
    //初始化
	IGXFactory::GetInstance().Init();
	try
	{
		do
		{
			//枚举设备
			gxdeviceinfo_vector vectorDeviceInfo;
			IGXFactory::GetInstance().UpdateDeviceList(1000, vectorDeviceInfo);
			if (0 == vectorDeviceInfo.size())
			{
				cout << "无可用设备!" << endl;
				break;
			}

			//打开第一台设备以及设备下面第一个流
			CGXDevicePointer ObjDevicePtr = IGXFactory::GetInstance().OpenDeviceBySN(vectorDeviceInfo[0].GetSN(), GX_ACCESS_EXCLUSIVE);
			CGXStreamPointer ObjStreamPtr = ObjDevicePtr->OpenStream(0);



			//注册设备掉线事件【目前只有千兆网系列相机支持此事件通知】
			GX_DEVICE_OFFLINE_CALLBACK_HANDLE hDeviceOffline = NULL;
			pDeviceOfflineEventHandler = new MyDeviceOfflineEventHandler();
			hDeviceOffline = ObjDevicePtr->RegisterDeviceOfflineCallback(pDeviceOfflineEventHandler, NULL);


			//获取远端设备属性控制器
			CGXFeatureControlPointer ObjFeatureControlPtr = ObjDevicePtr->GetRemoteFeatureControl();

			//设置曝光时间(示例中写死us,只是示例,并不代表真正可工作参数)

			//ObjFeatureControlPtr->GetFloatFeature("ExposureTime")->SetValue(50);

			//注册远端设备事件:曝光结束事件【目前只有千兆网系列相机支持曝光结束事件】

			//选择事件源

			ObjFeatureControlPtr->GetEnumFeature("EventSelector")->SetValue("ExposureEnd");

			//使能事件

			ObjFeatureControlPtr->GetEnumFeature("EventNotification")->SetValue("On");

			GX_FEATURE_CALLBACK_HANDLE hFeatureEvent = NULL;
			pFeatureEventHandler = new MyFeatureEventHandler();
			hFeatureEvent = ObjFeatureControlPtr->RegisterFeatureCallback("EventExposureEnd", pFeatureEventHandler, NULL);


			//注册回调采集

			pCaptureEventHandler = new MyCaptureEventHandler();
			ObjStreamPtr->RegisterCaptureCallback(pCaptureEventHandler, NULL);


			//发送开采命令

			ObjStreamPtr->StartGrab();
			ObjFeatureControlPtr->GetCommandFeature("AcquisitionStart")->Execute();
			CImageDataPointer objImageDataPtr;

			objImageDataPtr = ObjStreamPtr->GetImage(500);//超时时间使用500ms，用户可以自行设定

			if (objImageDataPtr->GetStatus() == GX_FRAME_STATUS_SUCCESS)

			{

				//采图成功而且是完整帧，可以进行图像处理...
				
				uint64_t nWidth = objImageDataPtr->GetWidth();

				uint64_t nHeight = objImageDataPtr->GetHeight();

				GX_PIXEL_FORMAT_ENTRY debug_emPixelFormat = objImageDataPtr->GetPixelFormat(); //获取当前图像格式，可参阅SDK文档7.13.5

				//假设原始数据不是Mono8图像
				void* pRaw8Buffer = NULL;
				pRaw8Buffer = objImageDataPtr->ConvertToRaw8(GX_BIT_0_7);
				camera_img_src = cv::Mat(nHeight, nWidth, CV_8U, (int *)pRaw8Buffer);



			}



			//此时开采成功,控制台打印信息,直到输入任意键继续
			getchar();

			//发送停采命令
			ObjFeatureControlPtr->GetCommandFeature("AcquisitionStop")->Execute();
			ObjStreamPtr->StopGrab();

			//注销采集回调
			ObjStreamPtr->UnregisterCaptureCallback();

			//注销远端设备事件
			ObjFeatureControlPtr->UnregisterFeatureCallback(hFeatureEvent);

			//注销设备掉线事件
			ObjDevicePtr->UnregisterDeviceOfflineCallback(hDeviceOffline);



			//释放资源

			ObjStreamPtr->Close();

			ObjDevicePtr->Close();

		} while (0);

	}

	catch (CGalaxyException&e)

	{

		cout << "错误码: " << e.GetErrorCode() << endl;

		cout << "错误描述信息: " << e.what() << endl;

	}

	catch (std::exception&e)

	{

		cout << "错误描述信息: " << e.what() << endl;

	}



	//反初始化库

	IGXFactory::GetInstance().Uninit();


	//销毁事件回调指针

	if (NULL != pCaptureEventHandler)
	{
		delete pCaptureEventHandler;
		pCaptureEventHandler = NULL;
	}

	if (NULL != pDeviceOfflineEventHandler)
	{
		delete pDeviceOfflineEventHandler;
		pDeviceOfflineEventHandler = NULL;
	}

	if (NULL != pFeatureEventHandler)
	{
		delete pFeatureEventHandler;
		pFeatureEventHandler = NULL;
	}

	return 0;

}


