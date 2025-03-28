#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include "Vzense_api2.h"
#include <thread>
#ifdef _WIN32
#include <windows.system.h>
#else
#include <sys/timeb.h>
#endif

#include "frame_capturer.h"

#pragma comment(lib,"vzense_api.lib")
//#pragma comment(lib,"opencv_world3416.lib")

using namespace std;
using namespace cv;

bool FrameCapturer::is_device_disconnected_ = true;

FrameCapturer::FrameCapturer()
{
	dataMode = PsDepthAndIR_30;
	depthRange = PsNearRange;
	wdrMode = { PsWDRTotalRange_Two, PsNearRange, 1, PsFarRange, 1, PsUnknown, 1 };
}
FrameCapturer::~FrameCapturer()
{

}

int FrameCapturer::SdkInit()
{
	PsReturnStatus status = Ps2_Initialize();
	if (status != PsReturnStatus::PsRetOK)
	{
		cout << "PsInitialize failed! error : " << status << endl;
		system("pause");
		return -1;
	}
	return 0;
}

uint32_t FrameCapturer::GetDeviceCount()
{
	delete pDeviceListInfo;
	deviceCount = 0;

	PsReturnStatus status = Ps2_GetDeviceCount(&deviceCount);
	if (status != PsReturnStatus::PsRetOK)
	{
		cout << "PsGetDeviceCount failed!" << endl;

#ifdef DCAM_550
		this_thread::sleep_for(chrono::seconds(1));
		goto GET;
#else
		//system("pause");
		return -1;
#endif

	}
	if (deviceCount <= 0)
	{
		//cout << "GetDeviceCount return 0" << endl;
		return 0;
	}

	cout << "Get device count: " << deviceCount << endl;

	pDeviceListInfo = new PsDeviceInfo[deviceCount];
	status = Ps2_GetDeviceListInfo(pDeviceListInfo, deviceCount);
	if (status != PsReturnStatus::PsRetOK)
	{
		cout << "Ps2_GetDeviceListInfo failed!" << endl;
		system("pause");
		return -1;
	}

	Ps2_SetHotPlugStatusCallback(HotPlugStateCallback);
	return deviceCount;
}

void FrameCapturer::HotPlugStateCallback(const PsDeviceInfo* pInfo, int status)
{
	if (status)
	{
		is_device_disconnected_ = true;
	}
	else
	{
		is_device_disconnected_ = false;
	}
	cout << "uri " << status << "  " << pInfo->uri << "    " << (status == 0 ? "add" : "remove") << endl;
	cout << "alia " << status << "  " << pInfo->alias << "    " << (status == 0 ? "add" : "remove") << endl;
}

int FrameCapturer::OpenDevice(uint32_t index,uint32_t session)
{
	PsReturnStatus status;
	status = Ps2_CloseDevice(&deviceHandle);
	//cout << "CloseDevice status: " << status << endl;

	sessionIndex = session;
	status = Ps2_OpenDevice(pDeviceListInfo[index].uri, &deviceHandle);
	if (status != PsReturnStatus::PsRetOK)
	{
		cout << "Ps2_OpenDevice failed! error : " << status << endl;
		//system("pause");
		return -1;
	}

	status = Ps2_StartStream(deviceHandle, sessionIndex);
	if (status != PsReturnStatus::PsRetOK)
	{
		cout << "StartStream failed!" << endl;
		//system("pause");
		return -1;
	}

	//PsCameraParameters cameraParameters;
	status = Ps2_GetCameraParameters(deviceHandle, sessionIndex, PsDepthSensor, &cameraParameters);

	cout << "Get PsGetCameraParameters status: " << status << endl;
	cout << "Depth Camera Intinsic: " << endl;
	cout << "Fx: " << cameraParameters.fx << endl;
	cout << "Cx: " << cameraParameters.cx << endl;
	cout << "Fy: " << cameraParameters.fy << endl;
	cout << "Cy: " << cameraParameters.cy << endl;
	cout << "Depth Distortion Coefficient: " << endl;
	cout << "K1: " << cameraParameters.k1 << endl;
	cout << "K2: " << cameraParameters.k2 << endl;
	cout << "P1: " << cameraParameters.p1 << endl;
	cout << "P2: " << cameraParameters.p2 << endl;
	cout << "K3: " << cameraParameters.k3 << endl;
	cout << "K4: " << cameraParameters.k4 << endl;
	cout << "K5: " << cameraParameters.k5 << endl;
	cout << "K6: " << cameraParameters.k6 << endl;

	//Get MeasuringRange
	//PsMeasuringRange measuringrange = { 0 };

	status = Ps2_GetDataMode(deviceHandle, sessionIndex, &dataMode);
	if (status != PsReturnStatus::PsRetOK)
		cout << "Ps2_GetDataMode failed!" << endl;
	else
		cout << "Get Ps2_GetDataMode : " << dataMode << endl;

	if (dataMode == PsWDR_Depth)
	{
		is_wdr_mode_ = true;

		status = Ps2_GetWDROutputMode(deviceHandle, sessionIndex, &wdrMode);

		if (status != PsReturnStatus::PsRetOK)
			cout << "Ps2_GetWDROutputMode failed!" << endl;
		else
		{
			if (wdrMode.totalRange == PsWDRTotalRange_Two)
			{
				depthRange = wdrMode.range2;
				status = Ps2_GetMeasuringRange(deviceHandle, sessionIndex, (PsDepthRange)wdrMode.range1, &measuringrange);
				if (status != PsReturnStatus::PsRetOK)
					cout << "Ps2_GetMeasuringRange failed!" << endl;
				else
				{
					switch ((PsDepthRange)wdrMode.range1)
					{
					case PsNearRange:
					case PsXNearRange:
					case PsXXNearRange:
						wdrRange1Slope = measuringrange.effectDepthMaxNear;
						break;

					case PsMidRange:
					case PsXMidRange:
					case PsXXMidRange:
						wdrRange1Slope = measuringrange.effectDepthMaxMid;
						break;

					case PsFarRange:
					case PsXFarRange:
					case PsXXFarRange:

						wdrRange1Slope = measuringrange.effectDepthMaxFar;
						break;
					default:
						break;
					}
					cout << "wdrRange1Slope   =  " << wdrRange1Slope << endl;
				}

				status = Ps2_GetMeasuringRange(deviceHandle, sessionIndex, (PsDepthRange)wdrMode.range2, &measuringrange);
				if (status != PsReturnStatus::PsRetOK)
					cout << "Ps2_GetMeasuringRange failed!" << endl;
				else
				{
					switch ((PsDepthRange)wdrMode.range2)
					{
					case PsNearRange:
					case PsXNearRange:
					case PsXXNearRange:
						wdrRange2Slope = wdrSlope == measuringrange.effectDepthMaxNear;
						break;

					case PsMidRange:
					case PsXMidRange:
					case PsXXMidRange:
						wdrRange2Slope = wdrSlope = measuringrange.effectDepthMaxMid;
						break;

					case PsFarRange:
					case PsXFarRange:
					case PsXXFarRange:

						wdrRange2Slope = wdrSlope = measuringrange.effectDepthMaxFar;
						break;
					default:
						break;
					}
					cout << "wdrSlope   =  wdrRange2Slope  " << wdrSlope << endl;
				}

			}
			else if (wdrMode.totalRange == PsWDRTotalRange_Three)
			{
				status = Ps2_GetMeasuringRange(deviceHandle, sessionIndex, (PsDepthRange)wdrMode.range1, &measuringrange);
				if (status != PsReturnStatus::PsRetOK)
					cout << "Ps2_GetMeasuringRange failed!" << endl;
				else
				{
					switch ((PsDepthRange)wdrMode.range1)
					{
					case PsNearRange:
					case PsXNearRange:
					case PsXXNearRange:
						wdrRange1Slope = measuringrange.effectDepthMaxNear;
						break;

					case PsMidRange:
					case PsXMidRange:
					case PsXXMidRange:
						wdrRange1Slope = measuringrange.effectDepthMaxMid;
						break;

					case PsFarRange:
					case PsXFarRange:
					case PsXXFarRange:

						wdrRange1Slope = measuringrange.effectDepthMaxFar;
						break;
					default:
						break;
					}
					cout << "wdrRange1Slope   =  " << wdrRange1Slope << endl;
				}


				status = Ps2_GetMeasuringRange(deviceHandle, sessionIndex, (PsDepthRange)wdrMode.range2, &measuringrange);
				if (status != PsReturnStatus::PsRetOK)
					cout << "Ps2_GetMeasuringRange failed!" << endl;
				else
				{
					switch ((PsDepthRange)wdrMode.range2)
					{
					case PsNearRange:
					case PsXNearRange:
					case PsXXNearRange:
						wdrRange2Slope = measuringrange.effectDepthMaxNear;
						break;

					case PsMidRange:
					case PsXMidRange:
					case PsXXMidRange:
						wdrRange2Slope = measuringrange.effectDepthMaxMid;
						break;

					case PsFarRange:
					case PsXFarRange:
					case PsXXFarRange:

						wdrRange2Slope = measuringrange.effectDepthMaxFar;
						break;
					default:
						break;
					}
					cout << "wdrRange2Slope   =  " << wdrSlope << endl;
				}
				status = Ps2_GetMeasuringRange(deviceHandle, sessionIndex, (PsDepthRange)wdrMode.range3, &measuringrange);
				if (status != PsReturnStatus::PsRetOK)
					cout << "Ps2_GetMeasuringRange failed!" << endl;
				else
				{
					switch ((PsDepthRange)wdrMode.range3)
					{
					case PsNearRange:
					case PsXNearRange:
					case PsXXNearRange:
						wdrRange3Slope = wdrSlope = measuringrange.effectDepthMaxNear;
						break;

					case PsMidRange:
					case PsXMidRange:
					case PsXXMidRange:
						wdrRange3Slope = wdrSlope = measuringrange.effectDepthMaxMid;
						break;

					case PsFarRange:
					case PsXFarRange:
					case PsXXFarRange:

						wdrRange3Slope = wdrSlope = measuringrange.effectDepthMaxFar;
						break;
					default:
						break;
					}
					cout << "wdrSlope   =  wdrRange3Slope  " << wdrSlope << endl;
				}
			}
		}
	}
	else
	{
		status = Ps2_GetDepthRange(deviceHandle, sessionIndex, &depthRange);
		if (status != PsReturnStatus::PsRetOK)
			cout << "Ps2_GetDepthRange failed!" << endl;
		else
			cout << "Get Depth Range " << depthRange << endl;

		status = Ps2_GetMeasuringRange(deviceHandle, sessionIndex, depthRange, &measuringrange);
		if (status != PsReturnStatus::PsRetOK)
			cout << "Ps2_GetMeasuringRange failed!" << endl;
		else
		{
			switch (depthRange)
			{
			case PsNearRange:
			case PsXNearRange:
			case PsXXNearRange:
				slope = measuringrange.effectDepthMaxNear;
				break;

			case PsMidRange:
			case PsXMidRange:
			case PsXXMidRange:
				slope = measuringrange.effectDepthMaxMid;
				break;

			case PsFarRange:
			case PsXFarRange:
			case PsXXFarRange:

				slope = measuringrange.effectDepthMaxFar;
				break;
			default:
				break;
			}
			cout << "slope  ==  " << slope << endl;
		}
	}


	//cv::Mat imageMat;
	//const string irImageWindow = "IR Image";
	//const string depthImageWindow = "Depth Image";
	//const string wdrDepthImageWindow = "WDR Depth Image";
	//const string wdrDepthRange1ImageWindow = "WDR Depth Range1 Image";
	//const string wdrDepthRange2ImageWindow = "WDR Depth Range2 Image";
	//const string wdrDepthRange3ImageWindow = "WDR Depth Range3 Image";

	//ofstream PointCloudWriter;
	//PsDepthVector3 DepthVector = { 0, 0, 0 };
	//PsVector3f WorldVector = { 0.0f };

	bool f_bPointClound = false;

	PsDepthRangeList rangelist = { 0 };
	int len = sizeof(rangelist);
	status = Ps2_GetProperty(deviceHandle, sessionIndex, PsPropertyDepthRangeList, &rangelist, &len);

	if (status == PsReturnStatus::PsRetOK && rangelist.count > 0)
	{
		cout << "Available Range List: ";
		for (int i = 0; i < rangelist.count - 1; i++)
		{
			cout << (int)rangelist.depthrangelist[i] << ",";
		}
		cout << (int)rangelist.depthrangelist[rangelist.count - 1] << endl;
	}
#ifndef DCAM_550

	//Enable the Depth and RGB synchronize feature
	Ps2_SetSynchronizeEnabled(deviceHandle, sessionIndex, true);

	//Set PixelFormat as PsPixelFormatBGR888 for opencv display
	Ps2_SetColorPixelFormat(deviceHandle, sessionIndex, PsPixelFormatBGR888);

	status = Ps2_GetCameraParameters(deviceHandle, sessionIndex, PsRgbSensor, &cameraParameters);

	cout << "Get PsGetCameraParameters status: " << status << endl;
	cout << "RGB Camera Intinsic: " << endl;
	cout << "Fx: " << cameraParameters.fx << endl;
	cout << "Cx: " << cameraParameters.cx << endl;
	cout << "Fy: " << cameraParameters.fy << endl;
	cout << "Cy: " << cameraParameters.cy << endl;
	cout << "RGB Distortion Coefficient: " << endl;
	cout << "K1: " << cameraParameters.k1 << endl;
	cout << "K2: " << cameraParameters.k2 << endl;
	cout << "K3: " << cameraParameters.k3 << endl;
	cout << "P1: " << cameraParameters.p1 << endl;
	cout << "P2: " << cameraParameters.p2 << endl;

	//PsCameraExtrinsicParameters CameraExtrinsicParameters;
	status = Ps2_GetCameraExtrinsicParameters(deviceHandle, sessionIndex, &CameraExtrinsicParameters);

	cout << "Get PsGetCameraExtrinsicParameters status: " << status << endl;
	cout << "Camera rotation: " << endl;
	cout << CameraExtrinsicParameters.rotation[0] << " "
		<< CameraExtrinsicParameters.rotation[1] << " "
		<< CameraExtrinsicParameters.rotation[2] << " "
		<< CameraExtrinsicParameters.rotation[3] << " "
		<< CameraExtrinsicParameters.rotation[4] << " "
		<< CameraExtrinsicParameters.rotation[5] << " "
		<< CameraExtrinsicParameters.rotation[6] << " "
		<< CameraExtrinsicParameters.rotation[7] << " "
		<< CameraExtrinsicParameters.rotation[8] << " "
		<< endl;

	cout << "Camera transfer: " << endl;
	cout << CameraExtrinsicParameters.translation[0] << " "
		<< CameraExtrinsicParameters.translation[1] << " "
		<< CameraExtrinsicParameters.translation[2] << " " << endl;

	//const string rgbImageWindow = "RGB Image";
	//const string mappedDepthImageWindow = "MappedDepth Image";
	//const string mappedRgbImageWindow = "MappedRGB Image";
	//bool f_bMappedRGB = true;
	//bool f_bMappedDepth = true;
	//bool f_bSync = false;

#endif
//	cout << "\n--------------------------------------------------------------------" << endl;
//	cout << "--------------------------------------------------------------------" << endl;
//	cout << "Press following key to set corresponding feature:" << endl;
//	cout << "0/1/2...: Change depth range Near/Middle/Far..." << endl;
//	cout << "P/p: Save point cloud data into PointCloud.txt in current directory" << endl;
//	cout << "T/t: Change background filter threshold value" << endl;
//#ifndef DCAM_550
//	cout << "S/s: Enable or disable the Depth and RGB synchronize feature " << endl;
//	cout << "M/m: Change data mode: input corresponding index in terminal:" << endl;
//	cout << "                    0: Output Depth and RGB in 30 fps" << endl;
//	cout << "                    1: Output IR and RGB in 30 fps" << endl;
//	cout << "                    2: Output Depth and IR in 30 fps" << endl;
//	cout << "                    3: Output Depth/IR frames alternatively in 15fps, and RGB in 30fps" << endl;
//	cout << "                    4: Output WDR_Depth and RGB in 30 fps" << endl;
//	cout << "R/r: Change the RGB resolution: input corresponding index in terminal:" << endl;
//	cout << "                             0: 1920*1080" << endl;
//	cout << "                             1: 1280*720" << endl;
//	cout << "                             2: 640*480" << endl;
//	cout << "                             3: 640*360" << endl;
//	cout << "Q/q: Enable or disable the mapped RGB in Depth space" << endl;
//	cout << "L/l: Enable or disable the mapped Depth in RGB space" << endl;
//	cout << "V/v: Enable or disable the WDR depth fusion feature " << endl;
//#else
//	cout << "M/m: Change data mode: input corresponding index in terminal:" << endl;
//	cout << "                    0: Output Depth in 30 fps" << endl;
//	cout << "                    1: Output IR in 30 fps" << endl;
//	cout << "                    2: Output Depth and IR in 30 fps" << endl;
//	cout << "                    3: Output WDR_Depth in 30 fps" << endl;
//#endif
//
//	cout << "Esc: Program quit " << endl;
//	cout << "--------------------------------------------------------------------" << endl;
//	cout << "--------------------------------------------------------------------\n" << endl;

	//int destroycount = 0;
	bool is_enabled = false;
	status = Ps2_GetMapperEnabledDepthToRGB(deviceHandle, sessionIndex, &is_enabled);

	status = Ps2_GetMapperEnabledRGBToDepth(deviceHandle, sessionIndex, &is_enabled);
	//is_device_disconnected_ = false;
	return slope;
}

int FrameCapturer::CaptureFrames()
{
	//PsFrameReady frameReady = { 0 };
	PsReturnStatus status;
	status = Ps2_ReadNextFrame(deviceHandle, sessionIndex, &frameReady);
	if (status != PsReturnStatus::PsRetOK)
	{
		cout << "CaptureFrames() failed" << endl;
		return -1;
	}
	return 0;
}

inline void FrameCapturer::OpencvDepth(uint32_t slope, int height, int width, uint8_t* pData, cv::Mat& dispImg)
{
	dispImg = cv::Mat(height, width, CV_16UC1, pData);
	//int type = dispImg.type();
//	Point2d pointxy(width / 2, height / 2);
//	int val = dispImg.at<ushort>(pointxy);
//	char text[20];
//#ifdef _WIN32
//	sprintf_s(text, "%d", val);
//#else
//	snprintf(text, sizeof(text), "%d", val);
//#endif
	//dispImg.convertTo(dispImg, CV_8U, 255.0 / slope);
	//applyColorMap(dispImg, dispImg, cv::COLORMAP_RAINBOW);
	//int color;
	//if (val > 2500)
	//	color = 0;
	//else
	//	color = 4096;
	//circle(dispImg, pointxy, 4, Scalar(color, color, color), -1, 8, 0);
	//putText(dispImg, text, pointxy, FONT_HERSHEY_DUPLEX, 2, Scalar(color, color, color));
}

inline void FrameCapturer::OpencvDepth16(uint32_t slope, int height, int width, uint8_t* pData, cv::Mat& dispImg)
{
	dispImg = cv::Mat(height, width, CV_16UC1, pData);
}

int FrameCapturer::GetDepthValue(cv::Point2d point_xy, cv::Mat& dispImg)
{
	int val = dispImg.at<ushort>(point_xy);
	return val;
}

int FrameCapturer::GetIrFrame(PsDeviceHandle handle, uint32_t session, cv::Mat& frame)
{
	if (1 == frameReady.ir)
	{
		PsReturnStatus status;
		PsFrame irFrame;
		status = Ps2_GetFrame(deviceHandle, sessionIndex, PsIRFrame, &irFrame);

		if (irFrame.pFrameData != NULL)
		{
#ifdef FPS	 
			countof_loop_ir++;
			if (countof_loop_ir >= FPS_LEN)
			{
				fps_ir = 1000 * FPS_LEN / tatoldelay_ir;
				//cout << fps_tof << endl;
				countof_loop_ir = 0;
				tatoldelay_ir = 0;
			}
#endif
			//Display the IR Image
			frame = cv::Mat(irFrame.height, irFrame.width, CV_16UC1, irFrame.pFrameData);

			// Convert 16bit IR pixel (max pixel value is 3840) to 8bit for display
			frame.convertTo(frame, CV_8U, 255.0 / 3840);
#ifdef FPS
			if (fps_ir != 0)
			{
				char fps[20];
#ifdef _WIN32
				sprintf_s(fps, "FPS: %d", fps_ir);
#else
				snprintf(fps, sizeof(fps), "FPS: %d", fps_ir);
#endif
				putText(imageMat, fps, Point2d(10, 20), FONT_HERSHEY_DUPLEX, 0.5, Scalar(255, 255, 255));
			}
#endif
			//cv::imshow(irImageWindow, frame);
		}
		else
		{
			cout << "Ps2_GetFrame PsIRFrame status:" << status << " pFrameData is NULL " << endl;
		}
	}
	return 0;
}

int FrameCapturer::GetWdrDepthFrame(PsDeviceHandle handle, uint32_t session, cv::Mat& frame)
{
	if (1 == frameReady.wdrDepth)
	{
		PsReturnStatus status;
		PsFrame wdrDepthFrame;
		status = Ps2_GetFrame(deviceHandle, sessionIndex, PsWDRDepthFrame, &wdrDepthFrame);
		if (wdrDepthFrame.pFrameData != NULL)
		{
#ifdef FPS			 
			countof_loop_tof++;
			if (countof_loop_tof >= FPS_LEN)
			{
				fps_tof = 1000 * FPS_LEN / tatoldelay_tof;
				//cout << fps_tof << endl;
				countof_loop_tof = 0;
				tatoldelay_tof = 0;
			}
#endif
			//if (f_bPointClound)
			//{
			//	PointCloudWriter.open("PointCloud.txt");
			//	PsFrame& srcFrame = wdrDepthFrame;
			//	const int len = srcFrame.width * srcFrame.height;
			//	PsVector3f* worldV = new PsVector3f[len];

			//	Ps2_ConvertDepthFrameToWorldVector(deviceHandle, sessionIndex, srcFrame, worldV); //Convert Depth frame to World vectors.

			//	for (int i = 0; i < len; i++)
			//	{
			//		if (worldV[i].z == 0 || worldV[i].z == 0xFFFF)
			//			continue; //discard zero points
			//		PointCloudWriter << worldV[i].x << "\t" << worldV[i].y << "\t" << worldV[i].z << std::endl;
			//	}
			//	delete[] worldV;
			//	worldV = NULL;
			//	std::cout << "Save point cloud successful in PointCloud.txt" << std::endl;
			//	PointCloudWriter.close();
			//	f_bPointClound = false;
			//}
			//Display the WDR Depth Image
			OpencvDepth(wdrSlope, wdrDepthFrame.height, wdrDepthFrame.width, wdrDepthFrame.pFrameData, frame);
#ifdef FPS
			if (fps_tof != 0)
			{
				char fps[20];
#ifdef _WIN32
				sprintf_s(fps, "FPS: %d", fps_tof);
#else
				snprintf(fps, sizeof(fps), "FPS: %d", fps_tof);
#endif
				putText(imageMat, fps, Point2d(10, 20), FONT_HERSHEY_DUPLEX, 0.5, Scalar(0, 0, 0));
			}
#endif
			//cv::imshow(wdrDepthImageWindow, imageMat);
		}
		else
		{
			cout << "Ps2_GetFrame PsWDRDepthFrame status:" << status << " pFrameData is NULL " << endl;
		}
	}
	return 0;
}

int FrameCapturer::GetRgbFrame(PsDeviceHandle handle, uint32_t session, cv::Mat& frame)
{
	if (1 == frameReady.rgb)
	{
		PsReturnStatus status;
		PsFrame rgbFrame;
		status = Ps2_GetFrame(deviceHandle, sessionIndex, PsRGBFrame, &rgbFrame);

		if (rgbFrame.pFrameData != NULL)
		{
#ifdef FPS
			countof_loop_rgb++;
			if (countof_loop_rgb >= FPS_LEN)
			{
				fps_rgb = 1000 * FPS_LEN / tatoldelay_rgb;
				//cout << fps_tof<<endl;
				countof_loop_rgb = 0;
				tatoldelay_rgb = 0;
			}
#endif
			//Display the RGB Image
			//Mat imageMat;
			//imageMat = cv::Mat(rgbFrame.height, rgbFrame.width, CV_8UC3, rgbFrame.pFrameData);
			//imageMat = cv::Mat(rgbFrame.height, rgbFrame.width, CV_8UC3, rgbFrame.pFrameData);
			frame = cv::Mat(rgbFrame.height, rgbFrame.width, CV_8UC3, rgbFrame.pFrameData);
			//imshow("get rgb", frame);
			//namedWindow("image rgb frame");
			//imshow("image rgb frame", imageMat);
			//waitKey(25);
#ifdef FPS
			if (fps_rgb != 0)
			{
				char fps[20];
#ifdef _WIN32
				sprintf_s(fps, "FPS: %d", fps_rgb);
#else
				snprintf(fps, sizeof(fps), "FPS: %d", fps_rgb);
#endif
				putText(imageMat, fps, Point2d(10, 20), FONT_HERSHEY_DUPLEX, 0.5, Scalar(0, 0, 0));
			}
#endif
		}
		else
		{
			cout << "Ps2_GetFrame PsRGBFrame status:" << status << " pFrameData is NULL " << endl;
		}
	}
	return 0;
}

int FrameCapturer::GetDepthFrame(PsDeviceHandle handle, uint32_t session, cv::Mat& frame)
{
	if (1 == frameReady.depth)
	{
		PsReturnStatus status;
		PsFrame depthFrame;
		status = Ps2_GetFrame(deviceHandle, sessionIndex, PsDepthFrame, &depthFrame);

		if (depthFrame.pFrameData != NULL)
		{

			//if (f_bPointClound)
			//{
			//	PointCloudWriter.open("PointCloud.txt");
			//	PsFrame& srcFrame = depthFrame;
			//	const int len = srcFrame.width * srcFrame.height;
			//	PsVector3f* worldV = new PsVector3f[len];

			//	Ps2_ConvertDepthFrameToWorldVector(deviceHandle, sessionIndex, srcFrame, worldV); //Convert Depth frame to World vectors.

			//	for (int i = 0; i < len; i++)
			//	{
			//		if (worldV[i].z == 0 || worldV[i].z == 0xFFFF)
			//			continue; //discard zero points
			//		PointCloudWriter << worldV[i].x << "\t" << worldV[i].y << "\t" << worldV[i].z << std::endl;
			//	}
			//	delete[] worldV;
			//	worldV = NULL;
			//	std::cout << "Save point cloud successful in PointCloud.txt" << std::endl;
			//	PointCloudWriter.close();
			//	f_bPointClound = false;
			//}
			

			//Display the Depth Image
			if (is_wdr_mode_ && dataMode == PsWDR_Depth)
			{
				if (depthFrame.depthRange == wdrMode.range1 && wdrMode.range1Count != 0)
				{
#ifdef FPS
					countof_loop_wdr1++;
					if (countof_loop_wdr1 >= FPS_LEN)
					{
						fps_wdr1 = 1000 * FPS_LEN / tatoldelay_wdr1;
						//cout << fps_tof<<endl;
						countof_loop_wdr1 = 0;
						tatoldelay_wdr1 = 0;
					}
#endif
					OpencvDepth(wdrRange1Slope, depthFrame.height, depthFrame.width, depthFrame.pFrameData, frame);
#ifdef FPS
					if (fps_wdr1 != 0)
					{
						char fps[20];
#ifdef _WIN32
						sprintf_s(fps, "FPS: %d", fps_wdr1);
#else
						snprintf(fps, sizeof(fps), "FPS: %d", fps_wdr1);
#endif
						putText(imageMat, fps, Point2d(10, 20), FONT_HERSHEY_DUPLEX, 0.5, Scalar(0, 0, 0));
					}
#endif
					//cv::imshow(wdrDepthRange1ImageWindow, imageMat);
				}
				else if (depthFrame.depthRange == wdrMode.range2 && wdrMode.range2Count != 0)
				{
#ifdef FPS
					countof_loop_wdr2++;
					if (countof_loop_wdr2 >= FPS_LEN)
					{
						fps_wdr2 = 1000 * FPS_LEN / tatoldelay_wdr2;
						//cout << fps_tof<<endl;
						countof_loop_wdr2 = 0;
						tatoldelay_wdr2 = 0;
					}
#endif
					OpencvDepth(wdrRange2Slope, depthFrame.height, depthFrame.width, depthFrame.pFrameData, frame);
#ifdef FPS
					if (fps_wdr2 != 0)
					{
						char fps[20];
#ifdef _WIN32
						sprintf_s(fps, "FPS: %d", fps_wdr2);
#else
						snprintf(fps, sizeof(fps), "FPS: %d", fps_wdr2);
#endif
						putText(imageMat, fps, Point2d(10, 20), FONT_HERSHEY_DUPLEX, 0.5, Scalar(0, 0, 0));
					}
#endif
					//cv::imshow(wdrDepthRange2ImageWindow, imageMat);
				}
				else if (depthFrame.depthRange == wdrMode.range3 && wdrMode.range3Count != 0)
				{
#ifdef FPS
					countof_loop_wdr3++;
					if (countof_loop_wdr3 >= FPS_LEN)
					{
						fps_wdr3 = 1000 * FPS_LEN / tatoldelay_wdr3;
						//cout << fps_tof<<endl;
						countof_loop_wdr3 = 0;
						tatoldelay_wdr3 = 0;
					}
#endif
					OpencvDepth(wdrRange3Slope, depthFrame.height, depthFrame.width, depthFrame.pFrameData, frame);
#ifdef FPS
					if (fps_wdr3 != 0)
					{
						char fps[20];
#ifdef _WIN32
						sprintf_s(fps, "FPS: %d", fps_wdr3);
#else
						snprintf(fps, sizeof(fps), "FPS: %d", fps_wdr3);
#endif
						putText(imageMat, fps, Point2d(10, 20), FONT_HERSHEY_DUPLEX, 0.5, Scalar(0, 0, 0));
					}
#endif
					//cv::imshow(wdrDepthRange3ImageWindow, imageMat);
				}
			}
			else
			{
#ifdef FPS
				countof_loop_tof++;
				if (countof_loop_tof >= FPS_LEN)
				{
					fps_tof = 1000 * FPS_LEN / tatoldelay_tof;
					//cout << fps_tof<<endl;
					countof_loop_tof = 0;
					tatoldelay_tof = 0;
				}
#endif
				OpencvDepth(slope, depthFrame.height, depthFrame.width, depthFrame.pFrameData, frame);
#ifdef FPS
				if (fps_tof != 0)
				{
					char fps[20];
#ifdef _WIN32
					sprintf_s(fps, "FPS: %d", fps_tof);
#else
					snprintf(fps, sizeof(fps), "FPS: %d", fps_tof);
#endif
					putText(imageMat, fps, Point2d(10, 20), FONT_HERSHEY_DUPLEX, 0.5, Scalar(0, 0, 0));
				}
#endif
				//if (!depthWriter.isOpened())
				//{
				//	depthWriter.open("D:/hsy/OpenCV_code/Vzense_SDK_Windows-master/Bin/x64/output_depth.mp4",
				//		VideoWriter::fourcc('X', '2', '6', '4'),
				//		30,
				//		Size(rgbFrame.width, rgbFrame.height),
				//		true);
				//	if (!depthWriter.isOpened())
				//	{
				//		cout << "video writer open failed" << endl;
				//		return -1;
				//	}
				//}
				//cv::imshow(rgbImageWindow, imageMat);

				//if (depthWriter.isOpened())
				//{
				//	cout << "d" << flush;
				//	depthWriter.write(imageMat);
				//}
			}
		}
		else
		{
			cout << "Ps2_GetFrame PsDepthFrame status:" << status << " pFrameData is NULL " << endl;
		}
	}
	return 0;
}

int FrameCapturer::GetMapped2RgbDepthFrame(PsDeviceHandle handle, uint32_t session, cv::Mat& frame)
{
	//if (1 == frameReady.mappedDepth)
	{
		PsReturnStatus status;
		PsFrame mappedDepthFrame;
		status = Ps2_GetFrame(deviceHandle, sessionIndex, PsMappedDepthFrame, &mappedDepthFrame);

		if (mappedDepthFrame.pFrameData != NULL)
		{
			//Display the MappedDepth Image
//			frame = cv::Mat(mappedDepthFrame.height, mappedDepthFrame.width, CV_16UC1, mappedDepthFrame.pFrameData);
//
//			Point2d pointxy(100 / 2, 100 / 2);
//			int val = frame.at<ushort>(pointxy);
//			char text[20];
//#ifdef _WIN32
//			sprintf_s(text, "%d", val);
//#else
//			snprintf(text, sizeof(text), "%d", val);
//#endif
//			frame.convertTo(frame, CV_8U, 255.0 / slope);
//			//applyColorMap(frame, frame, cv::COLORMAP_RAINBOW);
//			int color;
//			if (val > 2500)
//				color = 0;
//			else
//				color = 4096;
//			circle(frame, pointxy, 4, Scalar(color, color, color), -1, 8, 0);
//			putText(frame, text, pointxy, FONT_HERSHEY_DUPLEX, 2, Scalar(color, color, color));
//			//cv::Mat mappedDepthMat;
//			//frame.convertTo(mappedDepthMat, CV_8U, 255.0 / (is_wdr_mode_ ? wdrSlope : slope));
//			//cv::applyColorMap(mappedDepthMat, mappedDepthMat, cv::COLORMAP_RAINBOW);
//			//cv::imshow(mappedDepthImageWindow, mappedDepthMat);


			cv::Mat mappedDepthMat;
			frame = cv::Mat(mappedDepthFrame.height, mappedDepthFrame.width, CV_16UC1, mappedDepthFrame.pFrameData);

			//Point2d pointxy(100 / 2, 100 / 2);
			//int val = mappedDepthMat.at<ushort>(pointxy);
			//char text[20];
#ifdef _WIN32
			//sprintf_s(text, "%d", val);
#else
			snprintf(text, sizeof(text), "%d", val);
#endif
			//mappedDepthMat.convertTo(frame, CV_8U, 255.0 / slope);
			//applyColorMap(frame, frame, cv::COLORMAP_RAINBOW);
			//int color;
			//if (val > 2500)
			//	color = 0;
			//else
			//	color = 4096;
			//circle(frame, pointxy, 4, Scalar(color, color, color), -1, 8, 0);
			//putText(frame, text, pointxy, FONT_HERSHEY_DUPLEX, 2, Scalar(color, color, color));
			//cv::Mat mappedDepthMat;
			//frame.convertTo(mappedDepthMat, CV_8U, 255.0 / (is_wdr_mode_ ? wdrSlope : slope));
			//cv::applyColorMap(mappedDepthMat, mappedDepthMat, cv::COLORMAP_RAINBOW);
			//cv::imshow(mappedDepthImageWindow, mappedDepthMat);
		}
		else
		{
			cout << "Ps2_GetFrame PsMappedDepthFrame status:" << status << " pFrameData is NULL " << endl;
		}
	}
	//else
	//{

	//}
	return 0;
}

int FrameCapturer::GetMapped2DepthRgbFrame(PsDeviceHandle handle, uint32_t session, cv::Mat& frame)
{
	//if (1 == frameReady.mappedRGB)
	{
		PsReturnStatus status;
		PsFrame mappedRGBFrame;
		status = Ps2_GetFrame(deviceHandle, sessionIndex, PsMappedRGBFrame, &mappedRGBFrame);

		if (mappedRGBFrame.pFrameData != NULL)
		{
			//Display the MappedRGB Image
			frame = cv::Mat(mappedRGBFrame.height, mappedRGBFrame.width, CV_8UC3, mappedRGBFrame.pFrameData);
			//cv::imshow(mappedRgbImageWindow, imageMat);
		}
		else
		{
			cout << "Ps2_GetFrame PsMappedRGBFrame status:" << status << " pFrameData is NULL " << endl;
		}
	}
	return 0;
}


int FrameCapturer::SetDataMode(int index)
{
	PsReturnStatus status;
#ifdef DCAM_550
	switch (index)
	{
	case 0:
		t_datamode = PsDepth_30;
		break;
	case 1:
		t_datamode = PsIR_30;
		break;
	case 2:
		t_datamode = PsDepthAndIR_30;
		break;
	case 3:
		t_datamode = PsWDR_Depth;
		break;
	default:
		cout << "Unsupported data mode!" << endl;
		continue;
	}
#else
	switch (index)
	{
	case 0:
		t_datamode = PsDepthAndRGB_30;
		break;
	case 1:
		t_datamode = PsIRAndRGB_30;
		break;
	case 2:
		t_datamode = PsDepthAndIR_30;
		break;
	case 3:
		t_datamode = PsDepthAndIR_15_RGB_30;
		break;
	case 4:
		t_datamode = PsWDR_Depth;
		break;
	default:
		cout << "Unsupported data mode!" << endl;
	}
#endif			
	if (t_datamode == PsWDR_Depth)
	{
		Ps2_SetWDROutputMode(deviceHandle, sessionIndex, &wdrMode);
		is_wdr_mode_ = true;

	}
	else
	{
		is_wdr_mode_ = false;
	}
	if (dataMode == PsWDR_Depth && t_datamode != PsWDR_Depth)
	{
		status = Ps2_GetDepthRange(deviceHandle, sessionIndex, &depthRange);
		cout << "Get depth range," << " depthRange: " << depthRange << endl;
		if (status != PsRetOK)
		{
			cout << "Get depth range failed! " << endl;
		}
		else
		{
			status = Ps2_GetMeasuringRange(deviceHandle, sessionIndex, depthRange, &measuringrange);
			if (status != PsReturnStatus::PsRetOK)
				cout << "Ps2_GetMeasuringRange failed!" << endl;
			else
			{
				switch (depthRange)
				{
				case PsNearRange:
				case PsXNearRange:
				case PsXXNearRange:
					slope = measuringrange.effectDepthMaxNear;
					break;

				case PsMidRange:
				case PsXMidRange:
				case PsXXMidRange:
					slope = measuringrange.effectDepthMaxMid;
					break;

				case PsFarRange:
				case PsXFarRange:
				case PsXXFarRange:

					slope = measuringrange.effectDepthMaxFar;
					break;
				default:
					break;
				}
				cout << "slope  ==  " << slope << endl;
			}
		}
	}

	status = Ps2_SetDataMode(deviceHandle, sessionIndex, (PsDataMode)t_datamode);
	if (status != PsRetOK)
	{
		cout << "Ps2_SetDataMode  status" << status << endl;
	}
	else
	{
		dataMode = t_datamode;
	}
	//destroycount = 3;
#ifdef FPS
	delayT = 0;

	countof_loop_tof = 0;
	tatoldelay_tof = 0;
	fps_tof = 0;

	countof_loop_rgb = 0;
	tatoldelay_rgb = 0;
	fps_rgb = 0;

	countof_loop_ir = 0;
	tatoldelay_ir = 0;
	fps_ir = 0;

	countof_loop_wdr1 = 0;
	tatoldelay_wdr1 = 0;
	fps_wdr1 = 0;

	countof_loop_wdr2 = 0;
	tatoldelay_wdr2 = 0;
	fps_wdr2 = 0;

	countof_loop_wdr3 = 0;
	tatoldelay_wdr3 = 0;
	fps_wdr3 = 0;
#endif
	return 0;
}

int FrameCapturer::SetDepthRange(int depthRange)
{
	PsReturnStatus status;
	switch (depthRange)
	{
	case 0:
		depth_range_ = PsNearRange;
		slope = 1450;
		break;
	case 1:
		depth_range_ = PsMidRange;
		slope = 3000;
		break;
	case 2:
		depth_range_ = PsFarRange;
		slope = 4400;
		break;
	case 3:
		depth_range_ = PsXNearRange;
		slope = 4800;
		break;
	case 4:
		depth_range_ = PsXMidRange;
		slope = 5600;
		break;
	case 5:
		depth_range_ = PsXFarRange;
		slope = 7500;
		break;
	case 6:
		depth_range_ = PsXXNearRange;
		slope = 9600;
		break;
	case 7:
		depth_range_ = PsXXMidRange;
		slope = 11200;
		break;
	case 8:
		depth_range_ = PsXXFarRange;
		slope = 15000;
		break;
	default:
		cout << "Unsupported Range!" << endl;
		//continue;
	}
	status = Ps2_SetDepthRange(deviceHandle, sessionIndex, depth_range_);
	if (depth_range_ == PsNearRange)
		cout << "Set depth range to Near," << " status: " << status << endl;
	else if (depth_range_ == PsMidRange)
		cout << "Set depth range to Mid," << " status: " << status << endl;
	else if (depth_range_ == PsFarRange)
		cout << "Set depth range to Far," << " status: " << status << endl;
	else if (depth_range_ == PsXNearRange)
		cout << "Set depth range to XNearRange," << " status: " << status << endl;
	else if (depth_range_ == PsXMidRange)
		cout << "Set depth range to XMidRange," << " status: " << status << endl;
	else if (depth_range_ == PsXFarRange)
		cout << "Set depth range to XFarRange," << " status: " << status << endl;
	else if (depth_range_ == PsXXNearRange)
		cout << "Set depth range to XXNearRange," << " status: " << status << endl;
	else if (depth_range_ == PsXXMidRange)
		cout << "Set depth range to XXMidRange," << " status: " << status << endl;
	else if (depth_range_ == PsXXFarRange)
		cout << "Set depth range to XXFarRange," << " status: " << status << endl;

	if (status != PsRetOK)
	{
		cout << "Set depth range failed! " << endl;
	}

	status = Ps2_GetDepthRange(deviceHandle, sessionIndex, &depth_range_);
	cout << "Get depth range," << " depthRange: " << depth_range_ << endl;
	if (status != PsRetOK)
	{
		cout << "Get depth range failed! " << endl;
	}
	else
	{
		status = Ps2_GetMeasuringRange(deviceHandle, sessionIndex, depth_range_, &measuringrange);
		if (status != PsReturnStatus::PsRetOK)
			cout << "Ps2_GetMeasuringRange failed!" << endl;
		else
		{
			switch (depthRange)
			{
			case PsNearRange:
			case PsXNearRange:
			case PsXXNearRange:
				slope = measuringrange.effectDepthMaxNear;
				break;

			case PsMidRange:
			case PsXMidRange:
			case PsXXMidRange:
				slope = measuringrange.effectDepthMaxMid;
				break;

			case PsFarRange:
			case PsXFarRange:
			case PsXXFarRange:

				slope = measuringrange.effectDepthMaxFar;
				break;
			default:
				break;
			}
			cout << "slope  ==  " << slope << endl;
		}
	}
	return 0;
}

int FrameCapturer::SetTreshold(int threshold)
{
	PsReturnStatus status;
	//threshold_ += 10;
	threshold_ = threshold;
	if (threshold_ > 100)
	{
		threshold_ = 0;
	}
	status = Ps2_SetThreshold(deviceHandle, sessionIndex, threshold_);

	if (PsRetOK == status)
	{
		cout << "Set background threshold value: " << threshold_ << endl;
		return 0;
	}
	else
	{
		cout << "Set background threshold error,check if the datamode is WDR mode" << endl;
		return -1;
	}
}

int FrameCapturer::SetWdrStyle(bool style)
{
	PsReturnStatus status;
	if (wdr_style_ == style)
	{
		return 0;
	}
	else
	{
		wdr_style_ = style;
	}

	status = Ps2_SetWDRStyle(deviceHandle, sessionIndex, wdr_style_ ? PsWDR_ALTERNATION : PsWDR_FUSION);
	if (PsRetOK == status)
	{
		cout << "WDR image output " << (wdr_style_ ? "alternatively in multi range." : "Fusion.") << endl;
		//wdr_style_ = !wdr_style_;
	}
	else
	{
		cout << "Set WDR Style " << (wdr_style_ ? PsWDR_ALTERNATION : PsWDR_FUSION) << "  satus : " << status << endl;
	}
#ifdef FPS
	delayT = 0;

	countof_loop_tof = 0;
	tatoldelay_tof = 0;
	fps_tof = 0;

	countof_loop_rgb = 0;
	tatoldelay_rgb = 0;
	fps_rgb = 0;

	countof_loop_ir = 0;
	tatoldelay_ir = 0;
	fps_ir = 0;

	countof_loop_wdr1 = 0;
	tatoldelay_wdr1 = 0;
	fps_wdr1 = 0;

	countof_loop_wdr2 = 0;
	tatoldelay_wdr2 = 0;
	fps_wdr2 = 0;

	countof_loop_wdr3 = 0;
	tatoldelay_wdr3 = 0;
	fps_wdr3 = 0;
#endif
}

int FrameCapturer::SetSynchronize(bool is_sync)
{
	PsReturnStatus status;
	if (is_sync_ == is_sync)
	{
		return 0;
	}
	else
	{
		is_sync_ = is_sync;
	}
	status = Ps2_SetSynchronizeEnabled(deviceHandle, sessionIndex, is_sync_);
	if (status == PsRetOK)
	{
		cout << "Set Synchronize " << (is_sync_ ? "Enabled." : "Disabled.") << endl;
		//f_bSync = !f_bSync;
	}
}

int FrameCapturer::SetRgbResolution(int index)
{
	PsReturnStatus status;
	switch (index)
	{
	case 1:
		resolution_ = PsRGB_Resolution_1280_720;
		break;
	case 2:
		resolution_ = PsRGB_Resolution_640_480;
		break;
	case 3:
		resolution_ = PsRGB_Resolution_640_360;
		break;
	default:
		break;
	}
	status = Ps2_SetRGBResolution(deviceHandle, sessionIndex, resolution_);
	if (status != PsRetOK)
	{
		return -1;
	}
	return 0;
}

int FrameCapturer::SetMapperEnableRgb2Depth(bool is_enable)
{
	PsReturnStatus status;
	if (map_rgb_2_depth_ == is_enable)
	{
		return 0;
	}
	else
	{
		map_rgb_2_depth_ = is_enable;
	}
	status = Ps2_SetMapperEnabledRGBToDepth(deviceHandle, sessionIndex, map_rgb_2_depth_);
	if (status == PsRetOK)
	{
		cout << "Set Mapper RGBToDepth " << (map_rgb_2_depth_ ? "Enabled." : "Disabled.") << endl;
		bool is_enabled = true;
		status = Ps2_GetMapperEnabledDepthToRGB(deviceHandle, sessionIndex, &is_enabled);

		status = Ps2_GetMapperEnabledRGBToDepth(deviceHandle, sessionIndex, &is_enabled);
		return 0;
		//f_bMappedDepth = !f_bMappedDepth;
	}
	else
	{
		cout << "Set Mapper RGBToDepth failed" << endl;
		return -1;
	}
}

int FrameCapturer::SetMapperEnableDepth2Rgb(bool is_enable)
{
	PsReturnStatus status;
	if (map_depth_2_rgb_ == is_enable)
	{
		return 0;
	}
	else
	{
		map_depth_2_rgb_ = is_enable;
	}
	status = Ps2_SetMapperEnabledDepthToRGB(deviceHandle, sessionIndex, map_depth_2_rgb_);
	if (status == PsRetOK)
	{
		cout << "Set Mapper DepthToRGB " << (map_depth_2_rgb_ ? "Enabled." : "Disabled.") << endl;
		bool is_enabled = false;
		status = Ps2_GetMapperEnabledDepthToRGB(deviceHandle, sessionIndex, &is_enabled);

		status = Ps2_GetMapperEnabledRGBToDepth(deviceHandle, sessionIndex, &is_enabled);
		return 0;
		//f_bMappedRGB = !f_bMappedRGB;
	}
	else
	{
		cout << "Set Mapper DepthToRGB failed" << endl;
		return -1;
	}
}

int FrameCapturer::GetCapturedFrames(std::map<std::string, cv::Mat>& frame_map)
{
	int ret = 0;
	ret = CaptureFrames();
	if (ret != 0)
	{
		return 0;
	}
	if (1 == frameReady.rgb)
	{
		GetRgbFrame(deviceHandle, sessionIndex, frame_map["RGB_Frame"]);
		ret++;
	}
	else
	{
		frame_map["RGB_Frame"] = Mat();
	}

	if (1 == frameReady.depth)
	{
		GetDepthFrame(deviceHandle, sessionIndex, frame_map["Depth_Frame"]);
		ret++;
	}
	else
	{
		frame_map["Depth_Frame"] = Mat();
	}

	if (1 == frameReady.mappedRGB)
	{
		GetMapped2DepthRgbFrame(deviceHandle, sessionIndex, frame_map["MappedRGB_Frame"]);
		ret++;
	}
	else
	{
		frame_map["MappedRGB_Frame"] = Mat();
	}

	if (1 == frameReady.mappedDepth)
	{
		GetMapped2RgbDepthFrame(deviceHandle, sessionIndex, frame_map["MappedDepth_Frame"]);
		ret++;
	}
	else
	{
		frame_map["MappedDepth_Frame"] = Mat();
	}

	//if (1 == frameReady.mappedIR)
	//{
	//	GetMapped
	//	ret++;
	//}
	//else
	//{
	//	frame_map_["MappedIR_Frame"] = Mat();
	//}

	if (1 == frameReady.ir)
	{
		GetIrFrame(deviceHandle, sessionIndex, frame_map["IR_Frame"]);
		ret++;
	}
	else
	{
		frame_map["IR_Frame"] = Mat();
	}

	if (1 == frameReady.wdrDepth)
	{
		GetWdrDepthFrame(deviceHandle, sessionIndex, frame_map["WDRDepth_Frame"]);
		ret++;
	}
	else
	{
		frame_map["WDRDepth_Frame"] = Mat();
	}

	return ret;
}


//void FrameCapturer::GetConvertedDepthCoordinate(cv::Point& input, cv::Point& output)
//{
//	PsVector3f world{ input.x, input.y, 0 };
//	PsDepthVector3 depth;
//	Ps2_ConvertWorldToDepth(deviceHandle, sessionIndex, &world, &depth, 1, &cameraParameters);
//	output.x = depth.depthX;
//	output.y = depth.depthY;
//}

cv::Point FrameCapturer::GetConvertedDepthCoordinate(cv::Point input)
{
	Point output;
	PsVector3f world{ input.x, input.y, 0 };
	PsDepthVector3 depth;
	Ps2_ConvertWorldToDepth(deviceHandle, sessionIndex, &world, &depth, 1, &cameraParameters);
	output.x = depth.depthX;
	output.y = depth.depthY;
	return output;
}