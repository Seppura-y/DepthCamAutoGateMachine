#include "frame_proc_thread.h"

#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <vector>

//#pragma comment(lib,"opencv_world3416.lib")

#define TEST
//#define TEST_RGB
#define RGB

using namespace cv;
using namespace std;

//vector<Rect> AGM_Rect;
//vector<Rect> Area_Rect;
//vector<Rect> Entry_Rect;
//vector<Rect> Exit_Rect;
//vector<Rect> Safe_Rect;
//bool roi_found = false;

FrameProcThread::FrameProcThread()
{
	CapturerSdkInit();
	FrameMapInit();
}

FrameProcThread::~FrameProcThread()
{

}

void FrameProcThread::Exit()
{
	lock_guard<mutex> lock(mtx_);
	is_exit_ = true;
}

int FrameProcThread::CapturerSdkInit()
{
	int ret = capturer_.SdkInit();
	if (0 != ret)
	{
		cout << "Sdk Init failed" << endl;
		return -1;
	}
	else
	{
		is_init_ = true;
		return 0;
	}
}

unsigned char FrameProcThread::GetAreasStatus()
{
	lock_guard<mutex> lock(mtx_);
	return areas_status_;
}

uint32_t FrameProcThread::GetDeviceCount()
{
	uint32_t ret = capturer_.GetDeviceCount();
	if (0 == ret)
	{
		cout << "capturer_.GetDeviceCount() return 0" << endl;
		return 0;
	}
	return ret;
}

void FrameProcThread::SetSessionIndex(uint32_t index)
{
	lock_guard<mutex> lock(mtx_);
	session_index_ = index;
}

void FrameProcThread::SetDeviceIndex(int index)
{
	device_index_ = 0;
	is_dev_index_changed_ = true;
}

void FrameProcThread::SetDataMode(int mode)
{
	lock_guard<mutex> lock(mtx_);
	data_mode_ = mode;
}

void FrameProcThread::SetDepthRange(int depthRange)
{
	lock_guard<mutex> lock(mtx_);
	depth_range_ = depthRange;
}

void FrameProcThread::SetTreshold(int threshold)
{
	lock_guard<mutex> lock(mtx_);
	threshold_ = threshold;
}

void FrameProcThread::SetWdrStyle(bool style)
{
	lock_guard<mutex> lock(mtx_);
	wdr_style_ = style;
}

void FrameProcThread::SetSynchronize(bool is_sync)
{
	lock_guard<mutex> lock(mtx_);
	is_sync_ = is_sync;
}

void FrameProcThread::SetRgbResolution(int index)
{
	lock_guard<mutex> lock(mtx_);
	resolution_ = index;
}

void FrameProcThread::SetMapperEnableRgb2Depth(bool is_enable)
{
	lock_guard<mutex> lock(mtx_);
	map_rgb_2_depth_ = is_enable;
	capturer_.SetMapperEnableRgb2Depth(is_enable);
}

void FrameProcThread::SetMapperEnableDepth2Rgb(bool is_enable)
{
	lock_guard<mutex> lock(mtx_);
	map_depth_2_rgb_ = is_enable;
	capturer_.SetMapperEnableDepth2Rgb(is_enable);
}

int FrameProcThread::OpenDevice()
{
	lock_guard<mutex> lock(mtx_);
	int ret = 0;
	ret = capturer_.OpenDevice(device_index_, session_index_);
	if (ret <= 0)
	{
		cout << "open device ret <= 0 ,failed" << endl;
		return ret;
	}
	slope_ = ret;

	ret = capturer_.SetDataMode(data_mode_);
	if (0 != ret)
	{
		return ret;
	}

	ret = capturer_.SetDepthRange(depth_range_);
	if (0 != ret)
	{
		return ret;
	}

	ret = capturer_.SetRgbResolution(resolution_);
	if (0 != ret)
	{
		return ret;
	}

	ret = capturer_.SetTreshold(threshold_);
	if (0 != ret)
	{
		return ret;
	}

	ret = capturer_.SetSynchronize(is_sync_);
	if (0 != ret)
	{
		return ret;
	}

	ret = capturer_.SetMapperEnableRgb2Depth(true);
	if (0 != ret)
	{
		return ret;
	}

	ret = capturer_.SetMapperEnableDepth2Rgb(true);
	if (0 != ret)
	{
		return ret;
	}
	
	if (is_wdr_mode_)
	{
		ret = capturer_.SetWdrStyle(wdr_style_);
		if (0 != ret)
		{
			return ret;
		}
	}

	is_dev_index_changed_ = false;
	return 0;
}


int FrameProcThread::Start()
{
	static int i = 0;
	lock_guard<mutex> lock(mtx_);
	if (!is_init_)
	{
		cout << "thread is not initialized" << endl;
		return -1;
	}

	this->thread_id_ = i++;
	is_exit_ = false;

	thread_ = thread(&FrameProcThread::Loop, this);
	cout << "thread " << this_thread::get_id() << " : start" << endl;
	return 0;
}

void FrameProcThread::Stop()
{
	cout << "thread " << this->thread_id_ << " : request stop";
	is_exit_ = true;
	if (thread_.joinable())
	{
		thread_.join();
	}
	cout << "thread" << this->thread_id_ << " : stop";
}


void FrameProcThread::Loop()
{
	int ret = 0;
	vector<Point> input_points;
	vector<Point> output_points;
	input_points.push_back(Point(1, 2));
	while (!is_exit_)
	{
		//'Z'退出
		//ProcessKeys(1);
		while(device_count_ <= 0)
		{
			device_count_ = GetDeviceCount();
			cout << "get device count : " << device_count_ << endl;
			this_thread::sleep_for(chrono::seconds(1));
		}

		if (is_dev_index_changed_)
		{
			ret = OpenDevice();
			if (0 != ret)
			{
				cout << "Open device failed" << endl;
				this_thread::sleep_for(1ms);
				continue;
			}
		}


		if (capturer_.GetCapturedFrames(frame_map_) <= 0)
		{
			if (capturer_.GetConnectStatus())
			{
				is_dev_index_changed_ = true;
			}
			this_thread::sleep_for(25ms);
			continue;
		}

		Mat pic;
		if (!frame_map_["RGB_Frame"].empty() && !frame_map_["MappedDepth_Frame"].empty())
		{
			Mat pic_depth;
			Mat pic_rgb;
			pic_depth = frame_map_["MappedDepth_Frame"];
			if (!is_agm_areas_init_)
			{
				if (InitAgmAreas(pic_depth) != 0)
				{
					continue;
				}
			}

			if (!is_depth_source_init_)
			{
				pic_depth.copyTo(depth_source_img_);
				is_source_init_ = true;
			}

			pic_rgb = frame_map_["RGB_Frame"];
			imshow("RGB Frame", pic_rgb);
			if (!is_rgb_source_init_)
			{
				pic_rgb.copyTo(rgb_source_img_);
				cvtColor(rgb_source_img_, rgb_source_img_, cv::COLOR_BGR2GRAY);
				is_rgb_source_init_ = true;
			}

			FrameProcessRgbDepth(pic_rgb, pic_depth);
		}

		//if (!frame_map_["Depth_Frame"].empty())
		//{
		//	pic = frame_map_["Depth_Frame"];
		//	if (!is_agm_areas_init_)
		//	{
		//		if (InitAgmAreas(pic) != 0)
		//		{
		//			continue;
		//		}
		//	}
		//	if (!is_source_init_)
		//	{
		//		//GetSourceImageFromDepth(pic);
		//		pic.copyTo(cut_source_img_);
		//		is_source_init_ = true;
		//		//imshow("cut_source_img_", cut_source_img_);
		//	}
		//	//FrameProcessDepth(pic);
		//	FrameProcessTruncate(pic);
		//}

		//if (!frame_map_["MappedRGB_Frame"].empty())
		//{
		//	pic = frame_map_["MappedRGB_Frame"];
		//	imshow("MappedRGB_Frame", pic);
		//}

		//if (!frame_map_["MappedDepth_Frame"].empty())
		//{
		//	pic = frame_map_["MappedDepth_Frame"];
		//	imshow("MappedDepth_Frame", pic);
		//}

		waitKey(25);
	}
}

int FrameProcThread::ProcessKeys(int delay)
{
	unsigned char key = waitKey(delay);
	if (key == 'z' || key == 'Z')
	{
		is_exit_ = true;
		return -1;
	}
	if (key == 'Q' || key == 'q')
	{
		rgb_threshold_ += 2;
		if (rgb_threshold_ > 255)
		{
			rgb_threshold_ = 50;
		}
		cout << "rgb threshold : " << rgb_threshold_ << endl;
	}
	if (key == 'A' || key == 'a')
	{
		rgb_threshold_ -= 2;
		if (rgb_threshold_ < 0)
		{
			rgb_threshold_ = 50;
		}
		cout << "rgb threshold : " << rgb_threshold_ << endl;
	}
	if (key == 'W' || key == 'w')
	{
		depth_threshold_ += 2;
		if (depth_threshold_ > 255)
		{
			depth_threshold_ = 50;
		}
		cout << "depth threshold : " << depth_threshold_ << endl;
	}
	if (key == 'S' || key == 's')
	{
		depth_threshold_ -= 2;
		if (depth_threshold_ < 0)
		{
			depth_threshold_ = 50;
		}
		cout << "depth threshold : " << depth_threshold_ << endl;
	}
	if (key == 'E' || key == 'e')
	{
		mapped_rgb_threshold_ += 2;
		if (mapped_rgb_threshold_ > 255)
		{
			mapped_rgb_threshold_ = 50;
		}
		cout << "mapped rgb threshold : " << mapped_rgb_threshold_ << endl;
	}
	if (key == 'D' || key == 'd')
	{
		mapped_rgb_threshold_ -= 2;
		if (mapped_rgb_threshold_ < 0)
		{
			mapped_rgb_threshold_ = 50;
		}
		cout << "mapped rgb threshold : " << mapped_rgb_threshold_ << endl;
	}

	if (key == 'G' || key == 'g')
	{
		is_source_init_ = false;
	}

	if (key == 'T' || key == 't')
	{
		is_agm_areas_init_ = false;
	}

	if (key == 'P' || key == 'p')
	{
		device_index_++;
		if (device_index_ > 3)
		{
			device_index_ = 0;
			is_dev_index_changed_ = true;
		}
	}

	return 0;
}


int FrameProcThread::FillAreaPoly(Mat& input_frame)
{
	Mat mask_img = Mat::zeros(input_frame.rows, input_frame.cols, CV_8UC3);

	if (left_rect_.size() < 2 || center_rect_.size() < 2 || right_rect_.size() < 2)
	{
		return -1;
	}
	Point2i entry0[] = { Point2i(left_rect_[0].x,left_rect_[0].y),
		Point2i(left_rect_[0].x,left_rect_[0].y + left_rect_[0].height),
		Point2i(left_rect_[0].x + left_rect_[0].width,left_rect_[0].y + left_rect_[0].height),
		Point2i(left_rect_[0].x + left_rect_[0].width,left_rect_[0].y) };

	Point2i entry1[] = { Point2i(left_rect_[1].x,left_rect_[1].y),
		Point2i(left_rect_[1].x,left_rect_[1].y + left_rect_[1].height),
		Point2i(left_rect_[1].x + left_rect_[1].width,left_rect_[1].y + left_rect_[1].height),
		Point2i(left_rect_[1].x + left_rect_[1].width,left_rect_[1].y) };

	Point2i safe0[] = { Point2i(center_rect_[0].x,center_rect_[0].y),
		Point2i(center_rect_[0].x,center_rect_[0].y + center_rect_[0].height),
		Point2i(center_rect_[0].x + center_rect_[0].width,center_rect_[0].y + center_rect_[0].height),
		Point2i(center_rect_[0].x + center_rect_[0].width,center_rect_[0].y) };

	Point2i safe1[] = { Point2i(center_rect_[1].x,center_rect_[1].y),
		Point2i(center_rect_[1].x,center_rect_[1].y + center_rect_[1].height),
		Point2i(center_rect_[1].x + center_rect_[1].width,center_rect_[1].y + center_rect_[1].height),
		Point2i(center_rect_[1].x + center_rect_[1].width,center_rect_[1].y) };

	Point2i exit0[] = { Point2i(right_rect_[0].x,right_rect_[0].y),
		Point2i(right_rect_[0].x,right_rect_[0].y + right_rect_[0].height),
		Point2i(right_rect_[0].x + right_rect_[0].width,right_rect_[0].y + right_rect_[0].height),
		Point2i(right_rect_[0].x + right_rect_[0].width,right_rect_[0].y) };

	Point2i exit1[] = { Point2i(right_rect_[1].x,right_rect_[1].y),
		Point2i(right_rect_[1].x,right_rect_[1].y + right_rect_[1].height),
		Point2i(right_rect_[1].x + right_rect_[1].width,right_rect_[1].y + right_rect_[1].height),
		Point2i(right_rect_[1].x + right_rect_[1].width,right_rect_[1].y) };

	int size[] = { sizeof(entry0) / sizeof(Point2i) };
	const Point2i* pts = &entry0[0];
	fillPoly(mask_img, &pts, size, 1, Scalar(64, 0, 64));

	pts = &entry1[0];
	fillPoly(mask_img, &pts, size, 1, Scalar(0, 64, 64));

	pts = &safe0[0];
	fillPoly(mask_img, &pts, size, 1, Scalar(0, 64, 0));

	pts = &safe1[0];
	fillPoly(mask_img, &pts, size, 1, Scalar(0, 64, 0));

	pts = &exit0[0];
	fillPoly(mask_img, &pts, size, 1, Scalar(0, 64, 64));

	pts = &exit1[0];
	fillPoly(mask_img, &pts, size, 1, Scalar(64, 0, 64));

	add(input_frame, mask_img, input_frame);

	return 0;
}

int FrameProcThread::FillAreaPolyForTest(cv::Mat& input_frame)
{
	Mat mask_img = Mat::zeros(input_frame.rows, input_frame.cols, CV_8UC3);
	int rgb_y_offset = 50;
	int rgb_x_offset = 30;

	if (left_rect_.size() < 2 || center_rect_.size() < 2 || right_rect_.size() < 2)
	{
		return -1;
	}
	Point2i entry0[] = { Point2i(left_rect_[0].x + rgb_x_offset,left_rect_[0].y - rgb_y_offset),
		Point2i(left_rect_[0].x + rgb_x_offset,left_rect_[0].y + left_rect_[0].height - rgb_y_offset),
		Point2i(left_rect_[0].x + rgb_x_offset + left_rect_[0].width,left_rect_[0].y + left_rect_[0].height - rgb_y_offset),
		Point2i(left_rect_[0].x + rgb_x_offset + left_rect_[0].width,left_rect_[0].y - rgb_y_offset) };

	Point2i entry1[] = { Point2i(left_rect_[1].x + rgb_x_offset,left_rect_[1].y - rgb_y_offset),
		Point2i(left_rect_[1].x + rgb_x_offset,left_rect_[1].y + left_rect_[1].height - rgb_y_offset),
		Point2i(left_rect_[1].x + rgb_x_offset + left_rect_[1].width,left_rect_[1].y + left_rect_[1].height - rgb_y_offset),
		Point2i(left_rect_[1].x + rgb_x_offset + left_rect_[1].width,left_rect_[1].y - rgb_y_offset) };

	Point2i safe0[] = { Point2i(center_rect_[0].x + rgb_x_offset,center_rect_[0].y - rgb_y_offset),
		Point2i(center_rect_[0].x + rgb_x_offset,center_rect_[0].y + center_rect_[0].height - rgb_y_offset),
		Point2i(center_rect_[0].x + rgb_x_offset + center_rect_[0].width,center_rect_[0].y + center_rect_[0].height - rgb_y_offset),
		Point2i(center_rect_[0].x + rgb_x_offset + center_rect_[0].width,center_rect_[0].y - rgb_y_offset) };

	Point2i safe1[] = { Point2i(center_rect_[1].x + rgb_x_offset,center_rect_[1].y - rgb_y_offset),
		Point2i(center_rect_[1].x + rgb_x_offset,center_rect_[1].y + center_rect_[1].height - rgb_y_offset),
		Point2i(center_rect_[1].x + rgb_x_offset + center_rect_[1].width,center_rect_[1].y + center_rect_[1].height - rgb_y_offset),
		Point2i(center_rect_[1].x + rgb_x_offset + center_rect_[1].width,center_rect_[1].y - rgb_y_offset) };

	Point2i exit0[] = { Point2i(right_rect_[0].x + rgb_x_offset,right_rect_[0].y - rgb_y_offset),
		Point2i(right_rect_[0].x + rgb_x_offset,right_rect_[0].y + right_rect_[0].height - rgb_y_offset),
		Point2i(right_rect_[0].x + rgb_x_offset + right_rect_[0].width,right_rect_[0].y + right_rect_[0].height - rgb_y_offset),
		Point2i(right_rect_[0].x + rgb_x_offset + right_rect_[0].width,right_rect_[0].y - rgb_y_offset) };

	Point2i exit1[] = { Point2i(right_rect_[1].x + rgb_x_offset,right_rect_[1].y - rgb_y_offset),
		Point2i(right_rect_[1].x + rgb_x_offset,right_rect_[1].y + right_rect_[1].height - rgb_y_offset),
		Point2i(right_rect_[1].x + rgb_x_offset + right_rect_[1].width,right_rect_[1].y + right_rect_[1].height - rgb_y_offset),
		Point2i(right_rect_[1].x + rgb_x_offset + right_rect_[1].width,right_rect_[1].y - rgb_y_offset) };

	int size[] = { sizeof(entry0) / sizeof(Point2i) };
	const Point2i* pts = &entry0[0];
	fillPoly(mask_img, &pts, size, 1, Scalar(64, 0, 64));

	pts = &entry1[0];
	fillPoly(mask_img, &pts, size, 1, Scalar(0, 64, 64));

	pts = &safe0[0];
	fillPoly(mask_img, &pts, size, 1, Scalar(0, 64, 0));

	pts = &safe1[0];
	fillPoly(mask_img, &pts, size, 1, Scalar(0, 64, 0));

	pts = &exit0[0];
	fillPoly(mask_img, &pts, size, 1, Scalar(0, 64, 64));

	pts = &exit1[0];
	fillPoly(mask_img, &pts, size, 1, Scalar(64, 0, 64));

	add(input_frame, mask_img, input_frame);
	return 0;
}

int FrameProcThread::FillAreaPolyForRgb(cv::Mat& input_frame)
{
	Mat mask_img = Mat::zeros(input_frame.rows, input_frame.cols, CV_8UC3);

	if (left_rect_.size() < 2 || center_rect_.size() < 2 || right_rect_.size() < 2)
	{
		return -1;
	}
	Point2i entry0[] = { Point2i(rgb_areas_rect_[0].x,rgb_areas_rect_[0].y),
		Point2i(rgb_areas_rect_[0].x,rgb_areas_rect_[0].y + rgb_areas_rect_[0].height),
		Point2i(rgb_areas_rect_[0].x + rgb_areas_rect_[0].width,rgb_areas_rect_[0].y + rgb_areas_rect_[0].height),
		Point2i(rgb_areas_rect_[0].x + rgb_areas_rect_[0].width,rgb_areas_rect_[0].y) };

	Point2i entry1[] = { Point2i(rgb_areas_rect_[1].x,rgb_areas_rect_[1].y),
		Point2i(rgb_areas_rect_[1].x,rgb_areas_rect_[1].y + rgb_areas_rect_[1].height),
		Point2i(rgb_areas_rect_[1].x + rgb_areas_rect_[1].width,rgb_areas_rect_[1].y + rgb_areas_rect_[1].height),
		Point2i(rgb_areas_rect_[1].x + rgb_areas_rect_[1].width,rgb_areas_rect_[1].y) };

	Point2i safe0[] = { Point2i(rgb_areas_rect_[2].x,rgb_areas_rect_[2].y),
		Point2i(rgb_areas_rect_[2].x,rgb_areas_rect_[2].y + rgb_areas_rect_[2].height),
		Point2i(rgb_areas_rect_[2].x + rgb_areas_rect_[2].width,rgb_areas_rect_[2].y + rgb_areas_rect_[2].height),
		Point2i(rgb_areas_rect_[2].x + rgb_areas_rect_[2].width,rgb_areas_rect_[2].y) };

	Point2i safe1[] = { Point2i(rgb_areas_rect_[3].x,rgb_areas_rect_[3].y),
		Point2i(rgb_areas_rect_[3].x,rgb_areas_rect_[3].y + rgb_areas_rect_[3].height),
		Point2i(rgb_areas_rect_[3].x + rgb_areas_rect_[3].width,rgb_areas_rect_[3].y + rgb_areas_rect_[3].height),
		Point2i(rgb_areas_rect_[3].x + rgb_areas_rect_[3].width,rgb_areas_rect_[3].y) };

	Point2i exit0[] = { Point2i(rgb_areas_rect_[4].x,rgb_areas_rect_[4].y),
		Point2i(rgb_areas_rect_[4].x,rgb_areas_rect_[4].y + rgb_areas_rect_[4].height),
		Point2i(rgb_areas_rect_[4].x + rgb_areas_rect_[4].width,rgb_areas_rect_[4].y + rgb_areas_rect_[4].height),
		Point2i(rgb_areas_rect_[4].x + rgb_areas_rect_[4].width,rgb_areas_rect_[4].y) };

	Point2i exit1[] = { Point2i(rgb_areas_rect_[5].x,rgb_areas_rect_[5].y),
		Point2i(rgb_areas_rect_[5].x,rgb_areas_rect_[5].y + rgb_areas_rect_[5].height),
		Point2i(rgb_areas_rect_[5].x + rgb_areas_rect_[5].width,rgb_areas_rect_[5].y + rgb_areas_rect_[5].height),
		Point2i(rgb_areas_rect_[5].x + rgb_areas_rect_[5].width,rgb_areas_rect_[5].y) };

	int size[] = { sizeof(entry0) / sizeof(Point2i) };
	const Point2i* pts = &entry0[0];
	fillPoly(mask_img, &pts, size, 1, Scalar(64, 0, 64));

	pts = &entry1[0];
	fillPoly(mask_img, &pts, size, 1, Scalar(0, 64, 64));

	pts = &safe0[0];
	fillPoly(mask_img, &pts, size, 1, Scalar(0, 64, 0));

	pts = &safe1[0];
	fillPoly(mask_img, &pts, size, 1, Scalar(0, 64, 0));

	pts = &exit0[0];
	fillPoly(mask_img, &pts, size, 1, Scalar(0, 64, 64));

	pts = &exit1[0];
	fillPoly(mask_img, &pts, size, 1, Scalar(64, 0, 64));

	add(input_frame, mask_img, input_frame);
	return 0;
}

void FrameProcThread::FrameMapInit()
{
	frame_map_.clear();
	frame_map_.insert(make_pair("RGB_Frame", Mat()));
	frame_map_.insert(make_pair("IR_Frame", Mat()));
	frame_map_.insert(make_pair("Depth_Frame", Mat()));
	frame_map_.insert(make_pair("MappedRGB_Frame", Mat()));
	frame_map_.insert(make_pair("MappedDepth_Frame", Mat()));
	frame_map_.insert(make_pair("WDRDepth_Frame", Mat()));
}

void FrameProcThread::GetSourceImageFromDepth(Mat& image)
{
	cvtColor(image, source_img, cv::COLOR_GRAY2BGR);
	is_source_init_ = true;
}


int FrameProcThread::InitAgmAreas(Mat& input_frame)
{
	int width = input_frame.cols;
	int height = input_frame.rows;
	unsigned int count = 0;
	Mat element;
	Mat src_rgb_frame;
	Mat rgb_frame;
	Mat tresh_frame;
	Mat draw_frame;
	Mat u8_frame;
	vector<vector<Point>> contours;

	int offset_x_1 = 0;
	int offset_y_1 = 0;
	//int offset_x_2 = 0;
	//int offset_y_2 = 0;


	left_rect_.clear();
	center_rect_.clear();
	right_rect_.clear();

//test，去除实验室天花板黑框
#ifndef TEST
	input_frame.convertTo(u8_frame, CV_8U,255.0/slope_);
	cvtColor(u8_frame, src_rgb_frame, cv::COLOR_GRAY2BGR);
	threshold(u8_frame, tresh_frame, 144, 255, cv::THRESH_BINARY);
	findContours(tresh_frame, contours, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	for (; count < contours.size(); count++)
	{
		Rect rect_tmp = boundingRect(contours[count]);
		if (rect_tmp.width > 300 && rect_tmp.height > 300)
		{
			offset_x_1 = rect_tmp.x + 20;
			offset_y_1 = rect_tmp.y + 20;
			rect_tmp = Rect(Point(rect_tmp.x + 20, rect_tmp.y + 20), Point(rect_tmp.x + rect_tmp.width - 20, rect_tmp.y + rect_tmp.height - 20));
			roi = u8_frame(rect_tmp);
		}
	}
#else
	input_frame.convertTo(u8_frame, CV_8U, 255.0 / slope_);
	cvtColor(u8_frame, src_rgb_frame, cv::COLOR_GRAY2BGR);
	threshold(u8_frame, tresh_frame, 144, 255, cv::THRESH_BINARY);
	roi = u8_frame(Rect(Point(0,0),Point(input_frame.cols,input_frame.rows)));
#endif

	if (roi.empty())
	{
		return -1;
	}

	//真正的闸机区域
	cvtColor(roi, rgb_frame, cv::COLOR_GRAY2BGR);
	threshold(roi, tresh_frame, depth_threshold_, 255, cv::THRESH_BINARY);
	threshold(tresh_frame, tresh_frame, 100, 255, cv::THRESH_BINARY_INV);

	//element = getStructuringElement(MORPH_RECT, Size(3, 3));
	//erode(tresh_frame, tresh_frame, element);
	//dilate(tresh_frame, tresh_frame, element);

	contours.clear();
	findContours(tresh_frame, contours, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	roi.copyTo(draw_frame);
	//drawContours(draw_frame, contours, -1, Scalar(0, 0, 255), 2);
	//imshow("roi contours", draw_frame);
	count = 0;
	agm_rect_.clear();
	for (; count < contours.size(); count++)
	{
		Rect rect_tmp = boundingRect(contours[count]);
		//找出闸机的外接矩形, 400 x 30
		if (rect_tmp.width > 400 && rect_tmp.height > 30)
		{
			agm_rect_.push_back(rect_tmp);
			rectangle(rgb_frame, rect_tmp, Scalar(0, 255, 0), 2);
		}
	}

	//设置通行区域
	if (agm_rect_.size() >= 2)
	{
		Point2i left_top(0,0);
		Point2i right_bottom(0,0);
		Rect rect_tmp;

		for (int i = 0; i < agm_rect_.size(); i++)
		{
			rect_tmp = agm_rect_.at(i);
			//rect_tmp在图片的上半部分
			if (rect_tmp.y < roi.rows / 2 && rect_tmp.width > roi.cols / 2)
			{
				//设置尺寸rational
				agm_pixel_length_ = rect_tmp.width;
				//设置通行区顶点
				left_top = Point2i(rect_tmp.x + offset_x_1, rect_tmp.y + rect_tmp.height + offset_y_1);
			}
			else if (rect_tmp.y < roi.rows / 2 && rect_tmp.width < roi.cols / 2)
			{
				continue;
			}
			//rect_tmp在图片的下半部分
			else if(rect_tmp.y > roi.rows / 2 && rect_tmp.width > roi.cols / 2)
			{
				right_bottom = Point2i(rect_tmp.x + rect_tmp.width + offset_x_1, rect_tmp.y/* + rect_tmp.height*/ + offset_y_1);
			}
			else if (rect_tmp.y > roi.rows / 2 && rect_tmp.width < roi.cols / 2)
			{
				continue;
			}
		}

		//通行区域ROI矩形
		roi_rect = Rect(left_top, right_bottom);
	}
	else
	{
		return -1;
	}

	//设置高度阈值
#if 1	//根据图片中心点获取

	Point2d center_point(u8_frame.rows / 2, u8_frame.cols / 2);
#ifdef TEST
	center_point = Point2d(roi.rows / 2, roi.cols / 2);
#endif
	//int center_height = capturer_.GetDepthValue(center_point,input_frame);
	//int center_height = input_frame.at<ushort>(center_point);
	camera_height_ = input_frame.at<ushort>(center_point);
	camera_height_ = 3000;
	//height_threshold_ = center_height - 24000;
#elif 1	//根据闸机高度来设置
	Point2d ref_point(agm_rect_.at(0).width / 2, agm_rect_.at(1).height / 2);
	//int ref_height = capturer_.GetDepthValue(ref_point, u8_frame);
	int ref_height = input_frame.at<ushort>(ref_point);
	height_threshold_1 = ref_height;
#endif


	//通行区域分成6个子区域
	if (!roi_rect.empty())
	{
#ifdef TEST
		int total_width = roi_rect.width;
		int safe_width = total_width / 6;
		int width = (total_width - safe_width) / 4;
		int height = roi_rect.height;

		Point2i top_left(roi_rect.x, roi_rect.y);
		Point2i height_step(0, height);
		Point2i width_step(width, 0);
		Point2i safe_width_step(safe_width / 2, 0);
		 
		left_rect_.push_back(Rect2i(top_left + 0 * width_step - 2 * safe_width_step, top_left + 1 * width_step - 2 * safe_width_step + height_step));
		left_rect_.push_back(Rect2i(top_left + 1 * width_step - 2 * safe_width_step, top_left + 2 * width_step - 2 * safe_width_step + height_step));
		 
		center_rect_.push_back(Rect2i(top_left + 2 * width_step - 2 * safe_width_step, top_left + 2 * width_step + height_step));
		center_rect_.push_back(Rect2i(top_left + 2 * width_step, top_left + 2 * width_step + 2 * safe_width_step + height_step));

		right_rect_.push_back(Rect2i(top_left + 2 * width_step + 2 * safe_width_step, top_left + 2 * width_step + 2 * safe_width_step + width_step + height_step));
		right_rect_.push_back(Rect2i(top_left + 3 * width_step + 2 * safe_width_step, top_left + 4 * width_step + 2 * safe_width_step + height_step));
		 
		 
		 
		//for (int i = 1; i <= 6; i++)
		//{
		//	if (3 == i)
		//	{
		//		center_rect_.push_back(Rect2i(top_left + 2 * width_step - 2 * safe_width_step, top_left + 2 * width_step + 2 *safe_width_step + height_step));
		//	}
		//	else if (4 == i)
		//	{
		//		center_rect_.push_back(Rect2i(top_left + 2 * width_step + 2 * safe_width_step, top_left + 2 * width_step + 4 * safe_width_step + height_step));
		//	}
		//	else if (5 == i)
		//	{
		//		right_rect_.push_back(Rect2i(top_left + 2 * width_step + 2 * safe_width_step, top_left + 2 * width_step + 2 * safe_width_step + width_step + height_step));
		//	}
		//	else if (6 == i)
		//	{
		//		right_rect_.push_back(Rect2i(top_left + 3 * width_step + 2 * safe_width_step, top_left + 4 * width_step + 2 * safe_width_step + height_step));
		//	}
		//	//else if (i < 3)
		//	//{
		//	//	Area_Rect.push_back(Rect2i(top_left + (i - 1) * width_step, top_left + i * width_step + height_step));
		//	//}
		//	else if (2 == i)
		//	{
		//		left_rect_.push_back(Rect2i(top_left + (i - 1) * width_step - 2 * safe_width_step, top_left + i * width_step + height_step));
		//	}
		//	else if (1 == i)
		//	{
		//		left_rect_.push_back(Rect2i(top_left + (i - 1) * width_step - 2 * safe_width_step, top_left + i * width_step + height_step));
		//	}

		//	//六等分
		//	//Area_Rect.push_back(Rect2i(top_left + (i - 1) * width_step, top_left + i * width_step + height_step));
		//}
#else
		int total_width = roi_rect.width;
		int width = total_width / 10;
		int height = roi_rect.height;

		Point2i top_left(roi_rect.x, roi_rect.y);
		Point2i height_step(0, height);
		Point2i width_step(width, 0);

		int k = 1;
		for (; k <= 6; k++)
		{
			if (k < 3)
			{
				left_rect_.push_back(Rect2i(top_left + (k - 1) * 2 * width_step, top_left + k * 2 * width_step + height_step));
			}
			else if (3 == k || 4 == k)
			{
				center_rect_.push_back(Rect2i(top_left + 2 * 2 * width_step + (k - 3)*width_step, top_left + 2 * 2 * width_step + (k - 2)*width_step + height_step));
			}
			else if (k > 4)
			{
				right_rect_.push_back(Rect2i(top_left + (k - 2) * 2 * width_step, top_left + (k - 1) * 2 * width_step + height_step));
			}
		}
#endif

		areas_rect_.push_back(left_rect_[0]);
		areas_rect_.push_back(left_rect_[1]);
		areas_rect_.push_back(center_rect_[0]);
		areas_rect_.push_back(center_rect_[1]);
		areas_rect_.push_back(right_rect_[0]);
		areas_rect_.push_back(right_rect_[1]);

		outside_rect_.push_back(Rect2i(Point2i(0,left_rect_[0].y),Point2i(left_rect_[0].x,left_rect_[0].y + left_rect_[0].height)));
		outside_rect_.push_back(Rect2i(Point2i(right_rect_[1].x + right_rect_[1].width, right_rect_[1].y), Point2i(right_rect_[1].x + right_rect_[1].width + 500, right_rect_[1].y + right_rect_[0].height)));


		new_areas_rect_.push_back(Rect2i(Point2i(0, left_rect_[0].y), Point2i(left_rect_[0].x, left_rect_[0].y + left_rect_[0].height)));

		new_areas_rect_.push_back(left_rect_[0]);
		new_areas_rect_.push_back(left_rect_[1]);
		new_areas_rect_.push_back(center_rect_[0]);
		new_areas_rect_.push_back(center_rect_[1]);
		new_areas_rect_.push_back(right_rect_[0]);
		new_areas_rect_.push_back(right_rect_[1]);

		new_areas_rect_.push_back(Rect2i(Point2i(right_rect_[1].x + right_rect_[1].width, right_rect_[1].y), Point2i(right_rect_[1].x + right_rect_[1].width + 500, right_rect_[1].y + right_rect_[0].height)));
		

		//rgb_areas_rect_.push_back(Rect2i(Point(capturer_.GetConvertedDepthCoordinate(Point(left_rect_[0].x, left_rect_[0].y))),
		//													Point(capturer_.GetConvertedDepthCoordinate(Point(left_rect_[0].x+left_rect_[0].width, left_rect_[0].y + left_rect_[0].height)))));
		//rgb_areas_rect_.push_back(Rect2i(Point(capturer_.GetConvertedDepthCoordinate(Point(left_rect_[1].x, left_rect_[1].y))),
		//													Point(capturer_.GetConvertedDepthCoordinate(Point(left_rect_[1].x + left_rect_[1].width, left_rect_[1].y + left_rect_[1].height)))));

		//rgb_areas_rect_.push_back(Rect2i(Point(capturer_.GetConvertedDepthCoordinate(Point(center_rect_[0].x, center_rect_[0].y))),
		//													Point(capturer_.GetConvertedDepthCoordinate(Point(center_rect_[0].x + center_rect_[0].width, center_rect_[0].y + center_rect_[0].height)))));
		//rgb_areas_rect_.push_back(Rect2i(Point(capturer_.GetConvertedDepthCoordinate(Point(center_rect_[1].x, center_rect_[1].y))),
		//													Point(capturer_.GetConvertedDepthCoordinate(Point(center_rect_[1].x + center_rect_[1].width, center_rect_[1].y + center_rect_[1].height)))));

		//rgb_areas_rect_.push_back(Rect2i(Point(capturer_.GetConvertedDepthCoordinate(Point(right_rect_[0].x, right_rect_[0].y))),
		//													Point(capturer_.GetConvertedDepthCoordinate(Point(right_rect_[0].x + right_rect_[0].width, right_rect_[0].y + right_rect_[0].height)))));
		//rgb_areas_rect_.push_back(Rect2i(Point(capturer_.GetConvertedDepthCoordinate(Point(right_rect_[1].x, right_rect_[1].y))),
		//													Point(capturer_.GetConvertedDepthCoordinate(Point(right_rect_[1].x + right_rect_[1].width, right_rect_[1].y + right_rect_[1].height)))));
		is_agm_areas_init_ = true;
	}
	else
	{
		return -1;
	}

	return 0;
}


void FrameProcThread::GetTruncatedDepthFrame(cv::Mat& input_frame,int height)
{
	if (height > 0)
	{
		Mat output_frame;
		int channels = input_frame.channels();
		int rows = input_frame.rows;
		int cols = input_frame.cols * channels;

		//if (input_frame.isContinuous())
		//{
		//	cols *= rows;
		//	rows = 1;
		//}

		Point2d pointxy;
		int val;
		ushort* p;
		for (int i = 0; i < rows; i++)
		{
			p = input_frame.ptr<ushort>(i);
			for (int j = 0; j < cols; j++)
			{
				pointxy = Point2d(j, i);
				val = input_frame.at<ushort>(pointxy);
				if (val > camera_height_ - height)
				{
					p[j] = ushort(0);
				}
			}
		}
	}
}

int FrameProcThread::GetObjectHeight(cv::Mat& input_frame, cv::Mat& src_frame, int length, int height, std::deque<cv::Rect>& output_rects)
{

	Mat roi;
	Mat roi_src;
	Mat input_tmp;
	Mat source_tmp;
	Mat diff_frame;
	vector<vector<Point>> contours;
	int ret = 0;
	int proc_count = 0;
	int height_threshold = height + height_step_;
	int offset_x = 0;
	int offset_y = 0;
	//auto rect_end = --(output_rects.end());

	for (int i = 0; i < output_rects.size(); i++)
	{
		if (output_rects.at(i).width > length_threshold_ || output_rects.at(i).height > length_threshold_)
		{
			offset_x = output_rects.at(i).x;
			offset_y = output_rects.at(i).y;

			//cut_source_img_.copyTo(roi_src);
			input_frame.copyTo(roi);

			roi_src = src_frame(output_rects.at(i));
			roi = roi(output_rects.at(i));

			GetTruncatedDepthFrame(roi_src, height_threshold);
			GetTruncatedDepthFrame(roi, height_threshold);

			roi_src.convertTo(roi_src, CV_8U, 255.0 / slope_);
			roi.convertTo(roi, CV_8U, 255.0 / slope_);

			cvtColor(roi_src, roi_src, CV_GRAY2BGR);
			cvtColor(roi, roi, CV_GRAY2BGR);

			absdiff(roi_src, roi, diff_frame);

			cvtColor(diff_frame, diff_frame, COLOR_BGR2GRAY);
			threshold(diff_frame, diff_frame, 30, 255, THRESH_BINARY);

			findContours(diff_frame, contours, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

			if (contours.size() == 0)
			{
				break;
			}

			for (unsigned int i = 0; i < contours.size(); i++)
			{
				Rect rect_tmp = boundingRect(contours[i]);
				if (rect_tmp.width < 50 || rect_tmp.height < 50)
				{
					continue;
				}
				else
				{
					output_rects.push_back(Rect(Point(rect_tmp.x + offset_x /*- 10*/,
						rect_tmp.y + offset_y /*- 10*/),
						Point(rect_tmp.x + offset_x + rect_tmp.width /*+ 10*/,
							rect_tmp.y + offset_y + rect_tmp.height /*+ 10*/)));
					output_rects.pop_front();

					proc_count++;
				}
			}

		}
		else //不需处理并符合规则的rect出队列,并插入到队尾
		{
			if (output_rects.at(i) == *(--output_rects.end()))
			{
				continue;
			}
			auto rect = output_rects.front();
			output_rects.push_back(rect);
			output_rects.pop_front();
		}
	}

	if (ret = proc_count == 0)
	{
		return ret;
	}
	else
	{
		GetObjectHeight(input_frame, src_frame, length, height_threshold, output_rects);
	}
	return 0;
}

int FrameProcThread::GetObjectHeight(cv::Mat& input_frame, int length, int height, std::deque<cv::Rect>& output_rects)
{
	Mat roi;
	Mat roi_src;
	Mat input_tmp;
	Mat source_tmp;
	Mat diff_frame;
	vector<vector<Point>> contours;
	int ret = 0;
	int proc_count = 0;
	int height_threshold = height + height_step_;
	int offset_x = 0;
	int offset_y = 0;
	//auto rect_end = --(output_rects.end());

	for (int i = 0; i < output_rects.size(); i++)
	{
		if (output_rects.at(i).width > length_threshold_ || output_rects.at(i).height > length_threshold_)
		{
			offset_x = output_rects.at(i).x;
			offset_y = output_rects.at(i).y;

			cut_source_img_.copyTo(roi_src);
			input_frame.copyTo(roi);

			roi_src = roi_src(output_rects.at(i));
			roi = roi(output_rects.at(i));

			GetTruncatedDepthFrame(roi_src, height_threshold);
			GetTruncatedDepthFrame(roi, height_threshold);

			roi_src.convertTo(roi_src, CV_8U, 255.0 / slope_);
			roi.convertTo(roi, CV_8U, 255.0 / slope_);

			cvtColor(roi_src, roi_src, CV_GRAY2BGR);
			cvtColor(roi, roi, CV_GRAY2BGR);

			absdiff(roi_src, roi, diff_frame);

			cvtColor(diff_frame, diff_frame, COLOR_BGR2GRAY);
			threshold(diff_frame, diff_frame, 30, 255, THRESH_BINARY);

			findContours(diff_frame, contours, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

			if (contours.size() == 0)
			{
				output_rects.pop_front();
				continue;
			}

			for (unsigned int i = 0; i < contours.size(); i++)
			{
				Rect rect_tmp = boundingRect(contours[i]);
				if (rect_tmp.width < 50 || rect_tmp.height < 50)
				{
					continue;
				}
				else
				{
					output_rects.push_back(Rect(Point(rect_tmp.x + offset_x /*- 10*/,
																			rect_tmp.y + offset_y /*- 10*/),
																Point(rect_tmp.x + offset_x + rect_tmp.width /*+ 10*/,
																			rect_tmp.y + offset_y + rect_tmp.height /*+ 10*/)));
					proc_count++;
					//当前处理的矩形出队列
					output_rects.pop_front();
				}
			}
			////当前处理的矩形出队列
			//output_rects.pop_front();

		}
		else //不需处理并符合规则的rect出队列,并插入到队尾
		{
			if (output_rects.at(i) == *(--output_rects.end()))
			{
				continue;
			}
			auto rect = output_rects.front();
			output_rects.push_back(rect);
			output_rects.pop_front();
		}
	}

	if (ret = proc_count == 0)
	{
		return ret;
	}
	else
	{
		GetObjectHeight(input_frame, length, height_threshold, output_rects);
	}
	return 0;
}

int FrameProcThread::GetObjectHeight(cv::Mat& input_frame, int length, int height, cv::Rect& input_rect, std::deque<cv::Rect>& output_rects, std::set<cv::Point, PointSort>& height_points)
{
	Mat roi;
	Mat roi_src;
	Mat input_tmp;
	Mat source_tmp;
	Mat diff_frame;
	vector<vector<Point>> contours;
	int ret = 0;
	int proc_count = 0;
	int height_threshold = height + height_step_;
	int offset_x = 0;
	int offset_y = 0;

	if (input_rect.width > length_threshold_ || input_rect.height > length_threshold_)
	{
		{
			offset_x = input_rect.x;
			offset_y = input_rect.y;

			cut_source_img_.copyTo(roi_src);
			input_frame.copyTo(roi);

			roi_src = roi_src(input_rect);
			roi = roi(input_rect);

			GetTruncatedDepthFrame(roi_src, height_threshold);
			GetTruncatedDepthFrame(roi, height_threshold);

			roi_src.convertTo(roi_src, CV_8U, 255.0 / slope_);
			roi.convertTo(roi, CV_8U, 255.0 / slope_);

			cvtColor(roi_src, roi_src, CV_GRAY2BGR);
			cvtColor(roi, roi, CV_GRAY2BGR);

			absdiff(roi_src, roi, diff_frame);

			cvtColor(diff_frame, diff_frame, COLOR_BGR2GRAY);
			threshold(diff_frame, diff_frame, 30, 255, THRESH_BINARY);
		}

		findContours(diff_frame, contours, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

		if (contours.size() == 0)
		{
			//if (!roi_rect.empty() && !roi_rect.contains(Point(input_rect.x + input_rect.width / 2, input_rect.y + input_rect.height / 2)))
			//{
			//	return 0;
			//}
			output_rects.push_back(input_rect);
			return 0;
		}

		for (auto i = contours.begin(); i != contours.end(); ++i)
		{
			Rect rect_tmp = boundingRect(*i);
			if (rect_tmp.width < 30 || rect_tmp.height < 30)
			{
				//output_rects.push_back(input_rect);
				continue;
			}
			else
			{
				rect_tmp = Rect(Point(rect_tmp.x + offset_x, rect_tmp.y + offset_y),
											Point(rect_tmp.x + offset_x + rect_tmp.width, rect_tmp.y + offset_y + rect_tmp.height));
				GetObjectHeight(input_frame, length, height_threshold, rect_tmp, output_rects, height_points);
			}
		}

	}
	else //符合规则的rect插入队尾
	{
		//if (!roi_rect.empty() && !roi_rect.contains(Point(input_rect.x + input_rect.width / 2, input_rect.y + input_rect.height / 2)))
		//{
		//	return 0;
		//}
		output_rects.push_back(input_rect);
	}
	return 0;
}


int FrameProcThread::FrameProcessTruncate(cv::Mat& input_frame)
{
	{
		lock_guard<mutex> lock(mtx_);
		areas_status_ = 0;
		memset(area_status_, 0, sizeof(area_status_));
	}
	bool is_inside_area = false;
	int points_remains = 0;
	int points_distance_min = 0;
	int covered_max_index = -1;
	int covered_min_index = -1;
#ifndef TEST_RGB
	Mat output_rgb_frame;
	//int rgb_y_offset = 50;
	//int rgb_x_offset = 30;
	if (!frame_map_["RGB_Frame"].empty())
	{
		output_rgb_frame = frame_map_["RGB_Frame"];
	}
	FillAreaPolyForRgb(output_rgb_frame);
	imshow("filled rgb frame", output_rgb_frame);
#endif
	Mat src_depth_frame;
	Mat src_rgb_frame;
	Mat output_frame;
	Mat diff_frame;
	set<Point,PointSort> height_points;
	vector<vector<Point>> contours;
	vector<vector<Rect>> object_rect;
	object_rect.push_back(vector<Rect>());
	object_rect.push_back(vector<Rect>());
	object_rect.push_back(vector<Rect>());
	if (cut_source_img_.empty())
	{
		cout << "FrameProcessTruncate failed : source image is empty" << endl;
		return -1;
	}
	//不知为什么在运行过程中会被修改,找不到原因
	if (slope_ > 4400)
	{
		slope_ = 4400;
	}
	cut_source_img_.convertTo(src_rgb_frame, CV_8U, 255.0 / slope_);
	cvtColor(src_rgb_frame, src_rgb_frame, COLOR_GRAY2BGR);

	//imshow("src_input_frame", input_frame);
	input_frame.convertTo(output_frame, CV_8U, 255.0 / slope_);
	cvtColor(output_frame, output_frame, COLOR_GRAY2BGR);
	//imshow("input_frame", output_frame);


	absdiff(src_rgb_frame, output_frame, diff_frame);

	//imshow("diff_frame", diff_frame);

	cvtColor(diff_frame, diff_frame, COLOR_BGR2GRAY);

	threshold(diff_frame, diff_frame, 30, 255, THRESH_BINARY);


	Mat element;
	element = getStructuringElement(MORPH_RECT, Size(3, 3));
	//erode(diff_frame, diff_frame, element);
	//dilate(diff_frame, diff_frame, element);

	//imshow("thresh_diff_frame", diff_frame);

	findContours(diff_frame, contours, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);


	for (unsigned int cont = 0; cont < contours.size(); cont++)
	{
		is_inside_area = false;
		Rect rect_tmp = boundingRect(contours[cont]);
		if (rect_tmp.width < 50 || rect_tmp.height < 50)
		{
			continue;
		}

		int cur_x = rect_tmp.x;
		int cur_y = rect_tmp.y;
		int cur_width = rect_tmp.width;
		int cur_height = rect_tmp.height;

		bool status_updated = false;

		uint8_t status_tmp1 = 0;
		uint8_t status_tmp2 = 0;

		deque<Rect> proc_rects;

		int distance_x = 0;
		int distance_y = 0;

		int height_val = 0;
		int length_val = 0;
		int width_val = 0;
		int side_a = 0;
		int side_b = 0;
		int side_c = 0;
		Rect height_rect;
		int adult_count = 0;
		char text[100];

		/*
		|||||||||||		|		|		|		|		|		|||||||||||
		|||||0||||	1	|	2	|	3	|	4	|	5	|	6	|||||7||||
		|||||||||||		|		|		|		|		|		|||||||||||
		*/

		//通行方向 :  0-------------------->>7
		for (int i = 1; (i < 7 && !status_updated); i++)
		{
			//先找右边框
			if (new_areas_rect_[i].contains(Point(cur_x + cur_width, cur_y + cur_height / 2)))
			{
				is_inside_area = true;
#ifdef TEST_RGB
				for (auto ct : contours[cont])
				{
					ct.x += rgb_x_offset;
					ct.y -= rgb_y_offset;
				}
				drawContours(output_rgb_frame, contours, cont, Scalar(0, 0, 255), 2);
#endif
				drawContours(output_frame, contours, cont, Scalar(0, 0, 255),2);
				proc_rects.push_back(rect_tmp);
				covered_max_index = i - 1;
				status_tmp1 |= 1;
				for (int j = 1; j < i; j++)
				{
					//再找左边框
					if (new_areas_rect_[j].contains(Point(cur_x, cur_y + cur_height / 2)))
					{
						//两边都在同一个区
						if (i == j)
						{
							covered_min_index = i - 1;
							//area_status_[i] |= status_tmp1;
							status_updated = true;
							break;
						}
						//两边均在通行区内，更新状态
						covered_min_index = j - 1;
					}
				}

				//只有一边在通行区内，判断0区中是否包含另一边，否则就是物体从0区以外一直延申到包含已知边的区中
				if (!status_updated)
				{
					if (new_areas_rect_[0].contains(Point(cur_x, cur_y + cur_height / 2)))
					{
						covered_min_index = 0;
					}
				}

			}
		}

		/*
		|||||||||||		|		|		|		|		|		|||||||||||
		|||||0||||	1	|	2	|	3	|	4	|	5	|	6	|||||7||||
		|||||||||||		|		|		|		|		|		|||||||||||
		*/

		//通行方向  :  0<<--------------------7
		status_updated = false;
		for (int i = 1; (i < 7 && !status_updated); i++)
		{
			//先找左边框
			if (new_areas_rect_[i].contains(Point(cur_x, cur_y + cur_height / 2)))
			{
				is_inside_area = true;
#ifdef TEST_RGB
				for (auto ct : contours[cont])
				{
					ct.x += rgb_x_offset;
					ct.y -= rgb_y_offset;
				}
				drawContours(output_rgb_frame, contours, cont, Scalar(0, 0, 255), 2);
#endif
				drawContours(output_frame, contours, cont, Scalar(0, 0, 255),2);
				proc_rects.push_back(rect_tmp);
				status_tmp1 |= 1;
				covered_min_index = i - 1;
				for (int j = 0; j < 7 - i; j++)
				{
					int k = i + j;
					if (new_areas_rect_[k].contains(Point(cur_x + cur_width, cur_y + cur_height / 2)))
					{
						//两边都在同一个区
						if (i == k)
						{
							covered_max_index = i - 1;
							status_updated = true;
							break;
						}
						//两边均在通行区内
						covered_max_index = k - 1;
					}
				}

				//只有一边在通行区内，判断0区中是否包含另一边，否则就是物体从7区以外一直延申到包含已知边的区中
				if (!status_updated)
				{
					if (new_areas_rect_[7].contains(Point(cur_x + cur_width, cur_y + cur_height / 2)))
					{
						covered_max_index = 7;
					}
				}
			}
		}

#if 1
		//GetObjectHeight(input_frame, 50, height_step_, proc_rects);
		if (!proc_rects.empty())
		{
			GetObjectHeight(input_frame, 50, height_step_, proc_rects.at(0), proc_rects, height_points);
			for (auto it = proc_rects.begin(); it != proc_rects.end(); ++it)
			{
				if (it->width == rect_tmp.width && it->height == rect_tmp.height)
				{
					continue;
				}
				Point p(it->x + it->width / 2, it->y + it->height / 2);
				if (camera_height_ - input_frame.at<ushort>(p) > 1000)
				{
					height_points.insert(p);
#ifdef TEST_RGB
					Rect rt(Point(it->x + rgb_x_offset, it->y - rgb_y_offset), Point(it->x + rgb_x_offset  + it->width, it->y - rgb_y_offset + it->height));
					rectangle(output_rgb_frame, rt, Scalar(255, 0, 0), 2);
#endif
					rectangle(output_frame, *it, Scalar(255, 0, 0), 2);
				}
				else
				{
					continue;
				}
			}

			for (auto it = height_points.begin(); it != height_points.end(); ++it)
			{
				Point cur_point = Point(*it);
				if (rect_tmp.contains(cur_point))
				{
					points_remains++;
					if (height_points.size() > 2 && it != (--height_points.end()))
					{
						Point next_point = Point(*(++it));
						points_distance_min = (points_distance_min < abs(cur_point.x - next_point.x)) ? points_distance_min : abs(cur_point.x - next_point.x);
						points_distance_min = points_distance_min * agm_meters_length_ / agm_pixel_length_;
					}
				}
#ifdef TEST_RGB
				Point cur_rgb_point(cur_point.x + rgb_x_offset, cur_point.y - rgb_y_offset);
				circle(output_rgb_frame, cur_rgb_point, 2, Scalar(255, 255, 255), -1, 2, 0);
#endif
				circle(output_frame, cur_point, 2, Scalar(255, 255, 255), -1, 2, 0);

				side_c = input_frame.at<ushort>(*it);

				//高度
				height_val = camera_height_ - side_c;
				//sprintf_s(text, "height : %d cm", height_val / 10);
#ifdef TEST_RGB
				putText(output_rgb_frame, text, Point(it->x - 100 + rgb_x_offset, it->y - rgb_y_offset), FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0));
#endif
				//putText(output_frame, text, Point(it->x - 100, it->y), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255));
				
				length_val = rect_tmp.width;
				length_val = length_val * agm_meters_length_ / agm_pixel_length_;

				width_val = rect_tmp.height;
				width_val = width_val * agm_meters_length_ / agm_pixel_length_;
				//memset(text, 0, sizeof(text));
				sprintf_s(text, "%d cm x %d cm", width_val / 10, length_val / 10);
				putText(output_frame, text, Point(it->x - 100, it->y), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 0));

				memset(text, 0, sizeof(text));
				sprintf_s(text, "%d cm", height_val / 10 + 10);
				putText(output_frame,text, Point(it->x - 100, it->y + 20), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 0));
				if (height_val > height_threshold_)
				{
					status_tmp1 |= 1 << 1;
					adult_count++;
#ifdef TEST_RGB
					putText(output_rgb_frame, text, Point(it->x - 100 + rgb_x_offset, it->y - rgb_y_offset), FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0));
#endif
					putText(output_frame, "adult", Point(it->x - 100, it->y - 20), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 0));
				}
				else
				{
#ifdef TEST_RGB
					putText(output_rgb_frame, text, Point(it->x - 100 + rgb_x_offset, it->y - rgb_y_offset), FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0));
#endif
					putText(output_frame, "free", Point(it->x - 100, it->y - 20), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 0));
				}
			}
		}

		//当前外接矩形内含有超过两个高于1.2m的物体，且他们的最小水平距离小于1m
		if (points_remains > 1 && points_distance_min < 1000)
		{
			status_tmp1 |= 1 << 2;
		}

		for (int i = covered_min_index; i <= covered_max_index; i++)
		{
			area_status_[i] |= status_tmp1;
		}
		 
#elif 0
		for (int i = 0; i < proc_rects.size(); i++)
		{
			height_rect = proc_rects.at(i);
			//proc_rects.pop_front();

			//rectangle(output_frame, height_rect, Scalar(0, 0, 0));

			//x轴像素距离
			distance_x = height_rect.x + height_rect.width / 2 - input_frame.cols / 2;
			//y轴像素距离
			distance_y = height_rect.y + height_rect.height / 2 - input_frame.rows / 2;
			//目标中心点与摄像头中心点的距离 ：像素距离
			side_b = sqrt(distance_x * distance_x + distance_y + distance_y);
			//转为厘米单位
			side_b = side_b * agm_meters_length_ / agm_pixel_length_;
			//摄像头到矩形中心点的距离
			Point p(height_rect.x + height_rect.width / 2, height_rect.y + height_rect.height / 2);
			circle(output_frame, p, 2, Scalar(255, 255, 255), -1, 2, 0);
			//side_c = input_frame.at<ushort>(Point(height_rect.y + height_rect.height / 2, height_rect.x + height_rect.width / 2));
			side_c = input_frame.at<ushort>(p);
			side_a = sqrt(side_c * side_c - side_b * side_b);
			//高度
			height_val = camera_height_ - side_a;
			sprintf_s(text, "height : %d", height_val);
			putText(output_frame, text, Point(height_rect.x - 100, height_rect.y), FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0));
			memset(text, 0, sizeof(text));
			height_val = camera_height_ - side_c;
			sprintf_s(text, "height : %d", height_val);
			putText(output_frame, text, Point(height_rect.x - 100, height_rect.y + 20), FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0));
			memset(text, 0, sizeof(text));
			height_val = side_c;
			sprintf_s(text, "height : %d", height_val);
			putText(output_frame, text, Point(height_rect.x - 100, height_rect.y + 40), FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0));
			memset(text, 0, sizeof(text));

			//sprintf_s(text, "height : %d", height_val);
			//putText(output_frame, text, Point(height_rect.x-100, height_rect.y), FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0));

			if (height_val < height_threshold_)
			{
				status_tmp1 |= 1 << 1;
				adult_count++;
				rectangle(output_frame, height_rect, Scalar(255, 0, 0), 2);
				putText(output_frame, "adult", Point(height_rect.x - 100, height_rect.y - 40), FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0));
			}
			else
			{
				rectangle(output_frame, height_rect, Scalar(255, 0, 0), 2);
				putText(output_frame, "free", Point(height_rect.x - 100, height_rect.y - 40), FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0));
			}

			//rectangle(output_frame, height_rect, Scalar(0, 255, 0));
		}

		//超过2个物体高度在120cm以上
		if (adult_count >= 2)
		{
			status_tmp1 |= 1 << 2;
		}

		for (int i = covered_min_index; i <= covered_max_index; i++)
		{
			area_status_[i] |= status_tmp1;
		}
#endif

		//TEST : 画出结果
#ifdef TEST
		if (is_inside_area)
		{
			uint8_t tmp = 0;
			uint8_t left_mask = 0xc0;
			uint8_t right_mask = 0x0c;
			uint8_t center_mask = 0x30;

			stringstream strst;
			strst << "status : ";
			for (int i = 0; i < 6; i++)
			{

				if (area_status_[i])
				{
					strst << 1;
					tmp |= 1 << 7 - i;
				}
				else
				{
					strst << 0;
					tmp &= ~(1 << 7 - i);
				}
			}
			cout << strst.str() << endl;

			if (tmp & center_mask)
			{
				object_rect[1].push_back(rect_tmp);
			}
			else if (tmp & left_mask)
			{
				object_rect[0].push_back(rect_tmp);
			}
			else if (tmp & right_mask)
			{
				object_rect[2].push_back(rect_tmp);
			}
		}
	}

	Rect test_rect;
	for (int i = 0; i < object_rect[0].size(); i++)
	{
#ifdef TEST_RGB
		test_rect = Rect(Point(object_rect[0][i].x + rgb_x_offset, object_rect[0][i].y-rgb_y_offset), Point(object_rect[0][i].x + rgb_x_offset + object_rect[0][i].width,+object_rect[0][i].y+ object_rect[0][i].height-rgb_y_offset));
		rectangle(output_rgb_frame, test_rect, Scalar(0, 0, 255), 2);
#endif
		rectangle(output_frame, object_rect[0][i], Scalar(0, 0, 255), 2);
	}

	for (int i = 0; i < object_rect[1].size(); i++)
	{
#ifdef TEST_RGB
		test_rect = Rect(Point(object_rect[0][i].x + rgb_x_offset, object_rect[0][i].y - rgb_y_offset), Point(object_rect[0][i].x + rgb_x_offset + object_rect[0][i].width, +object_rect[0][i].y + object_rect[0][i].height - rgb_y_offset));
		rectangle(output_rgb_frame, test_rect, Scalar(0, 255, 0), 2);
#endif
		rectangle(output_frame, object_rect[1][i], Scalar(0, 255, 0), 2);
	}

	for (int i = 0; i < object_rect[2].size(); i++)
	{
#ifdef TEST_RGB
		test_rect = Rect(Point(object_rect[0][i].x + rgb_x_offset, object_rect[0][i].y - rgb_y_offset), Point(object_rect[0][i].x + rgb_x_offset + object_rect[0][i].width, +object_rect[0][i].y + object_rect[0][i].height - rgb_y_offset));
		rectangle(output_rgb_frame, test_rect, Scalar(255, 0, 0), 2);
#endif
		rectangle(output_frame, object_rect[2][i], Scalar(255, 0, 0), 2);
	}

#ifdef TEST_RGB
	FillAreaPolyForTest(output_rgb_frame);
#endif
	FillAreaPoly(output_frame);
	imshow("output_frame", output_frame);
#ifdef TEST_RGB
	imshow("output_rgb_frame", output_rgb_frame);
#endif
#else
}
#endif//TEST 
}


int FrameProcThread::FrameProcessRgbDepth(cv::Mat& input_rgb,cv::Mat& input_depth)
{
	{
		lock_guard<mutex> lock(mtx_);
		areas_status_ = 0;
		memset(area_status_, 0, sizeof(area_status_));
	}
	bool is_inside_area = false;
	int points_remains = 0;
	int points_distance_min = 0;
	int covered_max_index = -1;
	int covered_min_index = -1;

	Mat output_rgb_frame;
	Mat rgb_gray_frame;

	Mat output_depth_frame;
	Mat depth_gray_frame;

	Mat src_depth_frame;
	Mat src_rgb_frame;

	//Mat output_frame;
	Mat diff_frame;

	set<Point, PointSort> height_points;
	vector<vector<Point>> contours;
	vector<vector<Rect>> object_rect;
	object_rect.push_back(vector<Rect>());
	object_rect.push_back(vector<Rect>());
	object_rect.push_back(vector<Rect>());
	if (depth_source_img_.empty())
	{
		cout << "FrameProcessTruncate failed : source image is empty" << endl;
		return -1;
	}
	//不知为什么在运行过程中会被修改,找不到原因
	if (slope_ > 4400)
	{
		slope_ = 4400;
	}
	rgb_source_img_.copyTo(src_rgb_frame);
	input_rgb.copyTo(output_rgb_frame);

	absdiff(src_rgb_frame, output_rgb_frame, diff_frame);

	cvtColor(diff_frame, diff_frame, COLOR_BGR2GRAY);

	threshold(diff_frame, diff_frame, 30, 255, THRESH_BINARY);

	Mat element;
	element = getStructuringElement(MORPH_RECT, Size(3, 3));
	//erode(diff_frame, diff_frame, element);
	//dilate(diff_frame, diff_frame, element);

	findContours(diff_frame, contours, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

	for (unsigned int cont = 0; cont < contours.size(); cont++)
	{
		is_inside_area = false;
		Rect rect_tmp = boundingRect(contours[cont]);
		if (rect_tmp.width < 50 || rect_tmp.height < 50)
		{
			continue;
		}

		int cur_x = rect_tmp.x;
		int cur_y = rect_tmp.y;
		int cur_width = rect_tmp.width;
		int cur_height = rect_tmp.height;

		bool status_updated = false;

		uint8_t status_tmp1 = 0;
		uint8_t status_tmp2 = 0;

		deque<Rect> proc_rects;

		int distance_x = 0;
		int distance_y = 0;

		int height_val = 0;
		int length_val = 0;
		int side_a = 0;
		int side_b = 0;
		int side_c = 0;
		Rect height_rect;
		int adult_count = 0;
		char text[100];

		/*
		|||||||||||		|		|		|		|		|		|||||||||||
		|||||0||||	1	|	2	|	3	|	4	|	5	|	6	|||||7||||
		|||||||||||		|		|		|		|		|		|||||||||||
		*/

		//通行方向 :  0-------------------->>7
		for (int i = 1; (i < 7 && !status_updated); i++)
		{
			//先找右边框
			if (new_areas_rect_[i].contains(Point(cur_x + cur_width, cur_y + cur_height / 2)))
			{
				is_inside_area = true;
				drawContours(output_rgb_frame, contours, cont, Scalar(0, 0, 255), 2);
				proc_rects.push_back(rect_tmp);
				covered_max_index = i - 1;
				status_tmp1 |= 1;
				for (int j = 1; j < i; j++)
				{
					//再找左边框
					if (new_areas_rect_[j].contains(Point(cur_x, cur_y + cur_height / 2)))
					{
						//两边都在同一个区
						if (i == j)
						{
							covered_min_index = i - 1;
							//area_status_[i] |= status_tmp1;
							status_updated = true;
							break;
						}
						//两边均在通行区内，更新状态
						covered_min_index = j - 1;
					}
				}

				//只有一边在通行区内，判断0区中是否包含另一边，否则就是物体从0区以外一直延申到包含已知边的区中
				if (!status_updated)
				{
					if (new_areas_rect_[0].contains(Point(cur_x, cur_y + cur_height / 2)))
					{
						covered_min_index = 0;
					}
				}

			}
		}

		/*
		|||||||||||		|		|		|		|		|		|||||||||||
		|||||0||||	1	|	2	|	3	|	4	|	5	|	6	|||||7||||
		|||||||||||		|		|		|		|		|		|||||||||||
		*/

		//通行方向  :  0<<--------------------7
		status_updated = false;
		for (int i = 1; (i < 7 && !status_updated); i++)
		{
			//先找左边框
			if (new_areas_rect_[i].contains(Point(cur_x, cur_y + cur_height / 2)))
			{
				is_inside_area = true;
				drawContours(output_rgb_frame, contours, cont, Scalar(0, 0, 255), 2);
				proc_rects.push_back(rect_tmp);
				status_tmp1 |= 1;
				covered_min_index = i - 1;
				for (int j = 0; j < 7 - i; j++)
				{
					int k = i + j;
					if (new_areas_rect_[k].contains(Point(cur_x + cur_width, cur_y + cur_height / 2)))
					{
						//两边都在同一个区
						if (i == k)
						{
							covered_max_index = i - 1;
							status_updated = true;
							break;
						}
						//两边均在通行区内
						covered_max_index = k - 1;
					}
				}

				//只有一边在通行区内，判断0区中是否包含另一边，否则就是物体从7区以外一直延申到包含已知边的区中
				if (!status_updated)
				{
					if (new_areas_rect_[7].contains(Point(cur_x + cur_width, cur_y + cur_height / 2)))
					{
						covered_max_index = 7;
					}
				}
			}
		}

#if 1
		//GetObjectHeight(input_frame, 50, height_step_, proc_rects);
		if (!proc_rects.empty())
		{
			GetObjectHeight(input_depth, 50, height_step_, proc_rects.at(0), proc_rects, height_points);
			for (auto it = proc_rects.begin(); it != proc_rects.end(); ++it)
			{
				if (it->width == rect_tmp.width && it->height == rect_tmp.height)
				{
					continue;
				}
				Point p(it->x + it->width / 2, it->y + it->height / 2);
				if (camera_height_ - input_depth.at<ushort>(p) > 1100)
				{
					height_points.insert(p);
					rectangle(output_rgb_frame, *it, Scalar(255, 0, 0), 2);
				}
				else
				{
					continue;
				}
			}

			for (auto it = height_points.begin(); it != height_points.end(); ++it)
			{
				Point cur_point = Point(*it);
				if (rect_tmp.contains(cur_point))
				{
					points_remains++;
					if (height_points.size() > 2 && it != (--height_points.end()))
					{
						Point next_point = Point(*(++it));
						points_distance_min = (points_distance_min < abs(cur_point.x - next_point.x)) ? points_distance_min : abs(cur_point.x - next_point.x);
						points_distance_min = points_distance_min * agm_meters_length_ / agm_pixel_length_;
					}
				}
				circle(output_rgb_frame, cur_point, 2, Scalar(255, 255, 255), -1, 2, 0);

				side_c = input_depth.at<ushort>(*it);

				//高度
				height_val = camera_height_ - side_c;
				//sprintf_s(text, "height : %d cm", height_val / 10);
				//putText(output_frame, text, Point(it->x - 100, it->y), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255));

				length_val = rect_tmp.width;
				length_val = length_val * agm_meters_length_ / agm_pixel_length_;
				//memset(text, 0, sizeof(text));
				sprintf_s(text, "%d cm x %d cm", height_val / 10, length_val / 10);
				putText(output_rgb_frame, text, Point(it->x - 100, it->y), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255));

				if (height_val > height_threshold_)
				{
					status_tmp1 |= 1 << 1;
					adult_count++;
#ifdef RGB
					putText(output_rgb_frame, text, Point(it->x - 100, it->y), FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0));
#endif
					putText(output_rgb_frame, "adult", Point(it->x - 100, it->y - 20), FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0));
				}
				else
				{
#ifdef RGB
					putText(output_rgb_frame, text, Point(it->x - 100, it->y), FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0));
#endif
					putText(output_rgb_frame, "free", Point(it->x - 100, it->y - 20), FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0));
				}
			}
		}

		//当前外接矩形内含有超过两个高于1.2m的物体，且他们的最小水平距离小于1m
		if (points_remains > 1 && points_distance_min < 1000)
		{
			status_tmp1 |= 1 << 2;
		}

		for (int i = covered_min_index; i <= covered_max_index; i++)
		{
			area_status_[i] |= status_tmp1;
		}

#elif 0
		for (int i = 0; i < proc_rects.size(); i++)
		{
			height_rect = proc_rects.at(i);
			//proc_rects.pop_front();

			//rectangle(output_frame, height_rect, Scalar(0, 0, 0));

			//x轴像素距离
			distance_x = height_rect.x + height_rect.width / 2 - input_frame.cols / 2;
			//y轴像素距离
			distance_y = height_rect.y + height_rect.height / 2 - input_frame.rows / 2;
			//目标中心点与摄像头中心点的距离 ：像素距离
			side_b = sqrt(distance_x * distance_x + distance_y + distance_y);
			//转为厘米单位
			side_b = side_b * agm_meters_length_ / agm_pixel_length_;
			//摄像头到矩形中心点的距离
			Point p(height_rect.x + height_rect.width / 2, height_rect.y + height_rect.height / 2);
			circle(output_frame, p, 2, Scalar(255, 255, 255), -1, 2, 0);
			//side_c = input_frame.at<ushort>(Point(height_rect.y + height_rect.height / 2, height_rect.x + height_rect.width / 2));
			side_c = input_frame.at<ushort>(p);
			side_a = sqrt(side_c * side_c - side_b * side_b);
			//高度
			height_val = camera_height_ - side_a;
			sprintf_s(text, "height : %d", height_val);
			putText(output_frame, text, Point(height_rect.x - 100, height_rect.y), FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0));
			memset(text, 0, sizeof(text));
			height_val = camera_height_ - side_c;
			sprintf_s(text, "height : %d", height_val);
			putText(output_frame, text, Point(height_rect.x - 100, height_rect.y + 20), FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0));
			memset(text, 0, sizeof(text));
			height_val = side_c;
			sprintf_s(text, "height : %d", height_val);
			putText(output_frame, text, Point(height_rect.x - 100, height_rect.y + 40), FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0));
			memset(text, 0, sizeof(text));

			//sprintf_s(text, "height : %d", height_val);
			//putText(output_frame, text, Point(height_rect.x-100, height_rect.y), FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0));

			if (height_val < height_threshold_)
			{
				status_tmp1 |= 1 << 1;
				adult_count++;
				rectangle(output_frame, height_rect, Scalar(255, 0, 0), 2);
				putText(output_frame, "adult", Point(height_rect.x - 100, height_rect.y - 40), FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0));
			}
			else
			{
				rectangle(output_frame, height_rect, Scalar(255, 0, 0), 2);
				putText(output_frame, "free", Point(height_rect.x - 100, height_rect.y - 40), FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0));
			}

			//rectangle(output_frame, height_rect, Scalar(0, 255, 0));
		}

		//超过2个物体高度在120cm以上
		if (adult_count >= 2)
		{
			status_tmp1 |= 1 << 2;
		}

		for (int i = covered_min_index; i <= covered_max_index; i++)
		{
			area_status_[i] |= status_tmp1;
		}
#endif

		//TEST : 画出结果
#ifdef TEST
		if (is_inside_area)
		{
			uint8_t tmp = 0;
			uint8_t left_mask = 0xc0;
			uint8_t right_mask = 0x0c;
			uint8_t center_mask = 0x30;

			stringstream strst;
			strst << "status : ";
			for (int i = 0; i < 6; i++)
			{

				if (area_status_[i])
				{
					strst << 1;
					tmp |= 1 << 7 - i;
				}
				else
				{
					strst << 0;
					tmp &= ~(1 << 7 - i);
				}
			}
			cout << strst.str() << endl;

			if (tmp & center_mask)
			{
				object_rect[1].push_back(rect_tmp);
			}
			else if (tmp & left_mask)
			{
				object_rect[0].push_back(rect_tmp);
			}
			else if (tmp & right_mask)
			{
				object_rect[2].push_back(rect_tmp);
			}
		}
	}

	Rect test_rect;
	for (int i = 0; i < object_rect[0].size(); i++)
	{
#ifdef RGB
		test_rect = Rect(Point(object_rect[0][i].x, object_rect[0][i].y), Point(object_rect[0][i].x + object_rect[0][i].width, +object_rect[0][i].y + object_rect[0][i].height));
		rectangle(output_rgb_frame, test_rect, Scalar(0, 0, 255), 2);
#endif
		rectangle(output_rgb_frame, object_rect[0][i], Scalar(0, 0, 255), 2);
	}

	for (int i = 0; i < object_rect[1].size(); i++)
	{
#ifdef RGB
		test_rect = Rect(Point(object_rect[0][i].x, object_rect[0][i].y), Point(object_rect[0][i].x + object_rect[0][i].width, +object_rect[0][i].y + object_rect[0][i].height));
		rectangle(output_rgb_frame, test_rect, Scalar(0, 255, 0), 2);
#endif
		rectangle(output_rgb_frame, object_rect[1][i], Scalar(0, 255, 0), 2);
	}

	for (int i = 0; i < object_rect[2].size(); i++)
	{
#ifdef RGB
		test_rect = Rect(Point(object_rect[0][i].x, object_rect[0][i].y), Point(object_rect[0][i].x + object_rect[0][i].width, +object_rect[0][i].y + object_rect[0][i].height));
		rectangle(output_rgb_frame, test_rect, Scalar(255, 0, 0), 2);
#endif
		rectangle(output_rgb_frame, object_rect[2][i], Scalar(255, 0, 0), 2);
	}

#ifdef RGB
	FillAreaPolyForTest(output_rgb_frame);
#endif
	FillAreaPoly(output_rgb_frame);
	imshow("output_rgb_frame", output_rgb_frame);
#else
}
#endif//TEST 
}

int FrameProcThread::FrameProcessMappedRgb(cv::Mat& input_frame)
{
	vector<vector<Point>> contours;
	int width = input_frame.cols;
	int height = input_frame.rows;
	Mat element;
	Mat gray_frame;
	Mat mask_img;
	Mat draw_frame;
	Mat roi;
	Rect roi_rect;
	cvtColor(input_frame, gray_frame, cv::COLOR_BGR2GRAY);
	//imshow("gray frame", gray_frame);

	Mat tresh_frame;
	threshold(gray_frame, tresh_frame, mapped_rgb_threshold_/*50*/, 255, cv::THRESH_BINARY_INV);
	//imshow("mapped rgb thresh frame", tresh_frame);
	threshold(tresh_frame, tresh_frame, 254, 255, cv::THRESH_BINARY_INV);
	vector<Rect> boundingRectangle;
	findContours(tresh_frame, contours, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	unsigned int i = 0;
	for (i; i < contours.size(); i++)
	{
		Rect rect_tmp = boundingRect(contours[i]);
		if (rect_tmp.width > 200 && rect_tmp.height > 100)
		{
			roi = tresh_frame(rect_tmp);
			roi_rect = rect_tmp;
			break;
		}
	}
	if (i == 0)
	{
		return -1;
	}
	threshold(roi, roi, 254, 255, cv::THRESH_BINARY_INV);


	contours.clear();
	findContours(roi, contours, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

	i = 0;
	for (i; i < contours.size(); i++)
	{
		Rect rect_tmp = boundingRect(contours[i]);
		if (rect_tmp.width > 200 && rect_tmp.height > 100)
		{
			//roi = tresh_frame(rect_tmp);
			agm_rect_.push_back(rect_tmp);
		}
	}
	if (i == 0)
	{
		return -1;
	}

	roi.copyTo(draw_frame);
	drawContours(roi, contours, -1, Scalar(255, 255, 255), 2);
	imshow("roi", roi);

	i = 0;
	int size = agm_rect_.size();
	//for (i; i < AGM_Rect.size(); i++)
	//{
	//	Rect rect_tmp = Rect(Point(AGM_Rect.at(i).x + roi_rect.x,AGM_Rect.at(i).y + roi_rect.y),Point(AGM_Rect.at(i).x + roi_rect.width, AGM_Rect.at(i).y + roi_rect.height));
	//	rectangle(input_frame, rect_tmp, Scalar(0, 0, 255), 2);
	//}
	imshow("mapped rgb draw contours", input_frame);

	//findContours(roi, contours, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	//for (unsigned int i = 0; i < contours.size(); i++)
	//{
	//	Rect rect_tmp = boundingRect(contours[i]);
	//	rectangle(roi, rect_tmp, Scalar(0, 255, 0), 2);
	//}

	//roi.copyTo(draw_frame);
	//drawContours(draw_frame, contours, -1, Scalar(0, 0, 255), 2);


	//imshow("mapped rgb draw contours", roi);
}

int FrameProcThread::FrameProcessMappedDepth(Mat& input_frame)
{
	return 0;
}

int FrameProcThread::FrameProcessRgb(Mat& input_frame)
{
	return 0;
}

