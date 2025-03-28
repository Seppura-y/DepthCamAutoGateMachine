#pragma once
#include <map>
#include <mutex>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include "Vzense_api2.h"

class FrameCapturer
{
public:
	FrameCapturer();
	~FrameCapturer();
	static bool is_device_disconnected_;
	static bool GetConnectStatus() { return is_device_disconnected_; };

	int SdkInit();
	uint32_t GetDeviceCount();
	//int OpenDevice(uint32_t index,uint32_t session,PsDeviceHandle& handle);
	int OpenDevice(uint32_t index, uint32_t session);

	inline void OpencvDepth(uint32_t slope, int height, int width, uint8_t* pData, cv::Mat& dispImg);
	inline void OpencvDepth16(uint32_t slope, int height, int width, uint8_t* pData, cv::Mat& dispImg);
	int GetDepthValue(cv::Point2d point_xy, cv::Mat& dispIm);

	int CaptureFrames();
	int GetCapturedFrames(std::map<std::string, cv::Mat>& frame_map);

	int GetDepthFrame(PsDeviceHandle handle, uint32_t session, cv::Mat& frame);
	int GetIrFrame(PsDeviceHandle handle, uint32_t session, cv::Mat& frame);
	int GetWdrDepthFrame(PsDeviceHandle handle, uint32_t session, cv::Mat& frame);
	int GetRgbFrame(PsDeviceHandle handle, uint32_t session, cv::Mat& frame);
	int GetMapped2DepthRgbFrame(PsDeviceHandle handle, uint32_t session, cv::Mat& frame);
	int GetMapped2RgbDepthFrame(PsDeviceHandle handle, uint32_t session, cv::Mat& frame);

	int SetDataMode(int index);
	int SetDepthRange(int depthRange);
	int SetTreshold(int threshold);
	int SetWdrStyle(bool style);
	int SetSynchronize(bool is_sync);
	int SetRgbResolution(int index);
	int SetMapperEnableRgb2Depth(bool is_enable);
	int SetMapperEnableDepth2Rgb(bool is_enable);

	void set_session_index(uint32_t session) { sessionIndex = session; };

public:
	//void GetConvertedDepthCoordinate(cv::Point& input, cv::Point& output);
	cv::Point GetConvertedDepthCoordinate(cv::Point input);
protected:
	static void HotPlugStateCallback(const PsDeviceInfo* pInfo, int status);
private:
	uint32_t sessionIndex = 0;
	uint32_t deviceIndex = 0;
	uint32_t deviceCount = 0;
	uint32_t slope = 1450;
	uint32_t wdrSlope = 4400;
	uint32_t wdrRange1Slope = 1450;
	uint32_t wdrRange2Slope = 4400;
	uint32_t wdrRange3Slope = 6000;

	PsDeviceInfo* pDeviceListInfo = nullptr;
	PsDepthRange depthRange = PsNearRange;
	PsDataMode dataMode = PsDepthAndIR_30;
	PsWDROutputMode wdrMode = { PsWDRTotalRange_Two, PsNearRange, 1, PsFarRange, 1, PsUnknown, 1 };
	bool is_wdr_mode_ = false;
	//bool f_bWDRMode = false;
	//bool bWDRStyle = true;

	PsFrameReady frameReady = { 0 };
	PsMeasuringRange measuringrange = { 0 };

	PsDeviceHandle deviceHandle;
	PsCameraParameters cameraParameters;
	PsCameraExtrinsicParameters CameraExtrinsicParameters;

	PsDataMode t_datamode = PsDepthAndIR_30;
	PsDepthRange depth_range_ = PsFarRange;
	uint16_t threshold_ = 0;
	bool wdr_style_ = true;
	bool is_sync_ = true;
	PsResolution resolution_ = PsRGB_Resolution_640_360;
	bool map_rgb_2_depth_ = false;
	bool map_depth_2_rgb_ = false;

	std::mutex mtx_;
	std::map<std::string, cv::Mat> frame_map_;
	//cv::Mat imageMat;


};

