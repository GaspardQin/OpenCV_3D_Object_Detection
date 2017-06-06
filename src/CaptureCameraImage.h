#pragma once
//Using DaHeng SDK
#include <iostream>
#include "thread_variables.h"
using namespace std;
//���û���ǰ���úù���ͷ�ļ�Ŀ¼,��Ҫ����GalaxyIncludes.h

#include"GalaxyIncludes.h"



//�û��̳е����¼�������
class MyDeviceOfflineEventHandler: public IDeviceOfflineEventHandler
{
public:
	void DoOnDeviceOfflineEvent(void* pUserParam)
	{
		cout << "�յ��豸�����¼�!" << endl;
	}
};

 //�û��̳����Ը����¼�������
class MyFeatureEventHandler: public IFeatureEventHandler
{
public:
	void DoOnFeatureEvent(const GxIAPICPP::gxstring&strFeatureName, void* pUserParam)
	{
		cout << "�յ��ع�����¼�!" << endl;
	}
};
//�û��̳вɼ��¼�������

class MyCaptureEventHandler: public ICaptureEventHandler
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



int captureCamera()
{

	//�����¼��ص�����ָ��
	IDeviceOfflineEventHandler* pDeviceOfflineEventHandler = NULL;///<�����¼��ص�����
	IFeatureEventHandler* pFeatureEventHandler = NULL;///<Զ���豸�¼��ص�����
	ICaptureEventHandler* pCaptureEventHandler = NULL;///<�ɼ��ص�����
    //��ʼ��
	IGXFactory::GetInstance().Init();
	try
	{
		do
		{
			//ö���豸
			gxdeviceinfo_vector vectorDeviceInfo;
			IGXFactory::GetInstance().UpdateDeviceList(1000, vectorDeviceInfo);
			if (0 == vectorDeviceInfo.size())
			{
				cout << "�޿����豸!" << endl;
				break;
			}

			//�򿪵�һ̨�豸�Լ��豸�����һ����
			CGXDevicePointer ObjDevicePtr = IGXFactory::GetInstance().OpenDeviceBySN(vectorDeviceInfo[0].GetSN(), GX_ACCESS_EXCLUSIVE);
			CGXStreamPointer ObjStreamPtr = ObjDevicePtr->OpenStream(0);



			//ע���豸�����¼���Ŀǰֻ��ǧ����ϵ�����֧�ִ��¼�֪ͨ��
			GX_DEVICE_OFFLINE_CALLBACK_HANDLE hDeviceOffline = NULL;
			pDeviceOfflineEventHandler = new MyDeviceOfflineEventHandler();
			hDeviceOffline = ObjDevicePtr->RegisterDeviceOfflineCallback(pDeviceOfflineEventHandler, NULL);


			//��ȡԶ���豸���Կ�����
			CGXFeatureControlPointer ObjFeatureControlPtr = ObjDevicePtr->GetRemoteFeatureControl();

			//�����ع�ʱ��(ʾ����д��us,ֻ��ʾ��,�������������ɹ�������)

			//ObjFeatureControlPtr->GetFloatFeature("ExposureTime")->SetValue(50);

			//ע��Զ���豸�¼�:�ع�����¼���Ŀǰֻ��ǧ����ϵ�����֧���ع�����¼���

			//ѡ���¼�Դ

			ObjFeatureControlPtr->GetEnumFeature("EventSelector")->SetValue("ExposureEnd");

			//ʹ���¼�

			ObjFeatureControlPtr->GetEnumFeature("EventNotification")->SetValue("On");

			GX_FEATURE_CALLBACK_HANDLE hFeatureEvent = NULL;
			pFeatureEventHandler = new MyFeatureEventHandler();
			hFeatureEvent = ObjFeatureControlPtr->RegisterFeatureCallback("EventExposureEnd", pFeatureEventHandler, NULL);


			//ע��ص��ɼ�

			pCaptureEventHandler = new MyCaptureEventHandler();
			ObjStreamPtr->RegisterCaptureCallback(pCaptureEventHandler, NULL);


			//���Ϳ�������

			ObjStreamPtr->StartGrab();
			ObjFeatureControlPtr->GetCommandFeature("AcquisitionStart")->Execute();
			CImageDataPointer objImageDataPtr;

			objImageDataPtr = ObjStreamPtr->GetImage(500);//��ʱʱ��ʹ��500ms���û����������趨

			if (objImageDataPtr->GetStatus() == GX_FRAME_STATUS_SUCCESS)

			{

				//��ͼ�ɹ�����������֡�����Խ���ͼ����...
				
				uint64_t nWidth = objImageDataPtr->GetWidth();

				uint64_t nHeight = objImageDataPtr->GetHeight();

				GX_PIXEL_FORMAT_ENTRY debug_emPixelFormat = objImageDataPtr->GetPixelFormat(); //��ȡ��ǰͼ���ʽ���ɲ���SDK�ĵ�7.13.5

				//����ԭʼ���ݲ���Mono8ͼ��
				void* pRaw8Buffer = NULL;
				pRaw8Buffer = objImageDataPtr->ConvertToRaw8(GX_BIT_0_7);
				camera_img_src = cv::Mat(nHeight, nWidth, CV_8U, (int *)pRaw8Buffer);



			}



			//��ʱ���ɳɹ�,����̨��ӡ��Ϣ,ֱ���������������
			getchar();

			//����ͣ������
			ObjFeatureControlPtr->GetCommandFeature("AcquisitionStop")->Execute();
			ObjStreamPtr->StopGrab();

			//ע���ɼ��ص�
			ObjStreamPtr->UnregisterCaptureCallback();

			//ע��Զ���豸�¼�
			ObjFeatureControlPtr->UnregisterFeatureCallback(hFeatureEvent);

			//ע���豸�����¼�
			ObjDevicePtr->UnregisterDeviceOfflineCallback(hDeviceOffline);



			//�ͷ���Դ

			ObjStreamPtr->Close();

			ObjDevicePtr->Close();

		} while (0);

	}

	catch (CGalaxyException&e)

	{

		cout << "������: " << e.GetErrorCode() << endl;

		cout << "����������Ϣ: " << e.what() << endl;

	}

	catch (std::exception&e)

	{

		cout << "����������Ϣ: " << e.what() << endl;

	}



	//����ʼ����

	IGXFactory::GetInstance().Uninit();


	//�����¼��ص�ָ��

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


