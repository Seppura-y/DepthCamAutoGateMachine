#pragma once
#include<thread>
#include <mutex>
#include <vector>
#include <set>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

#include "frame_capturer.h"

class FrameProcThread
{
public:
	FrameProcThread();
	~FrameProcThread();

	int Start();
	void Stop();
	void Exit();

	uint32_t GetDeviceCount();
	void SetDeviceIndex(int index);
	unsigned char GetAreasStatus();

	void SetSessionIndex(uint32_t index);
	void SetDataMode(int mode);
	void SetDepthRange(int depthRange);
	void SetTreshold(int threshold);
	void SetWdrStyle(bool style);
	void SetSynchronize(bool is_sync);
	void SetRgbResolution(int index);
	void SetMapperEnableRgb2Depth(bool is_enable);
	void SetMapperEnableDepth2Rgb(bool is_enable);

	//int OpenDevice(uint32_t index);
protected:
	void Loop();

	int CapturerSdkInit();
	int OpenDevice();
	void FrameMapInit();
	void GetSourceImageFromDepth(cv::Mat& image);

	int ProcessKeys(int delay);
	int InitAgmAreas(cv::Mat& input_frame);
	int FillAreaPoly(cv::Mat& input_frame);
	int FillAreaPolyForTest(cv::Mat& input_frame);
	int FillAreaPolyForRgb(cv::Mat& input_frame);

	int FrameProcessRgb(cv::Mat& input_frame);
	//int FrameProcessDepth(cv::Mat& input_frame);
	int FrameProcessMappedRgb(cv::Mat& input_frame);
	int FrameProcessMappedDepth(cv::Mat& input_frame);

	//int FrameProcessDepth1(cv::Mat& input_frame);

private:
	std::mutex mtx_;
	std::thread thread_;

	bool is_exit_ = true;
	bool is_init_ = false;
	int thread_id_ = -1;

	//摄像头属性
	uint16_t threshold_ = 0;
	bool is_wdr_mode_ = false;
	bool wdr_style_ = true;
	bool is_sync_ = true;
	bool map_rgb_2_depth_ = false;
	bool map_depth_2_rgb_ = false;
	int data_mode_ = 0;		//PsDepthAndRGB_30
	int depth_range_ = 2;	//PsFarRange
	int resolution_ = 3;		//PsRGB_Resolution_640_360

	//录像

	//不同来源图像的二值化阈值
	int rgb_threshold_ = 120;
	int depth_threshold_ = 126;
	int mapped_rgb_threshold_ = 50;


	//图像处理
	bool is_roi_init_ = false;
	bool is_source_init_ = false;
	bool is_agm_areas_init_ = false;

	cv::Mat roi;
	cv::Mat source_img;
	cv::Rect roi_rect;
	std::vector<cv::Rect> agm_rect_;
	std::vector<cv::Rect> areas_rect_;
	std::vector<cv::Rect> new_areas_rect_;
	std::vector<cv::Rect> left_rect_;
	std::vector<cv::Rect> center_rect_;
	std::vector<cv::Rect> right_rect_;
	std::vector<cv::Rect> outside_rect_;

	unsigned char areas_status_;

	//打开设备
	bool is_dev_index_changed_ = true;
	uint32_t device_count_ = 0;
	uint32_t device_index_ = 0;
	uint32_t session_index_ = 0;

	FrameCapturer capturer_;
	std::map<std::string, cv::Mat> frame_map_;



	//新增截取高度处理
	cv::Mat cut_source_img_;
	uint16_t height_threshold_ = 1200;
	uint16_t length_threshold_ = 50;
	uint16_t width_threshold_ = 50;
	uint16_t height_step_ = 50;

	uint32_t slope_ = 0;

	uint8_t area_status_[6] = { 0 };
	
	int agm_meters_length_ = 1800;
	int agm_pixel_length_ = 0;
	int camera_height_ = 0;

	class PointSort
	{
	public:
		bool operator() (const cv::Point& p1, const cv::Point& p2) const
		{
			return p1.x < p2.x || (p1.x == p2.x && p1.y < p2.y);
		}
	};
	class RectSort
	{
	public:
		bool operator()(const cv::Rect& r1, const cv::Rect& r2)const
		{
			return r1.x + r1.width / 2 < r2.x + r2.width / 2 || (r1.x + r1.width / 2 == r2.x + r2.width / 2 && r1.y + r1.height / 2 < r2.y + r2.height / 2);
		}
	};

	int FrameProcessTruncate(cv::Mat& input_frame);
	int FrameProcessRgbDepth(cv::Mat& input_rgb, cv::Mat& input_depth);
	int GetObjectHeight(cv::Mat& input_frame, int length, int height, std::deque<cv::Rect>& output_rects);
	//int GetObjectHeight(cv::Mat& input_frame, int length, int height, std::deque<cv::Rect>& output_rects, std::set<cv::Point,PointSort>& height_points);
	//int GetObjectHeight(cv::Mat& input_frame, int length, int height, std::deque<cv::Rect>::iterator it, std::deque<cv::Rect>& output_rects, std::set<cv::Point, PointSort>& height_points);
	int GetObjectHeight(cv::Mat& input_frame, int length, int height,cv::Rect& input_rect, std::deque<cv::Rect>& output_rects, std::set<cv::Point, PointSort>& height_points);
	int GetObjectHeight(cv::Mat& input_frame, cv::Mat& src_frame, int length, int height, std::deque<cv::Rect>& output_rects);
	void GetTruncatedDepthFrame(cv::Mat& input_frame,int height);

	std::vector<cv::Rect> rgb_areas_rect_;
	bool is_depth_source_init_ = false;
	bool is_rgb_source_init_ = false;
	cv::Mat rgb_source_img_;
	cv::Mat depth_source_img_;
};

