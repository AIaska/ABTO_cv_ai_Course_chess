#pragma once

#include "opencv2/calib3d.hpp"
#include<chrono>
#include <list>

#include "Frame.h"


namespace ISXCheckersDetector
{
	class CheckersDetector
	{
	public:
		CheckersDetector(int num_of_chosen_camera = 0)
		{
			//load num_of_chosen_camera-th camera
			camera.open(num_of_chosen_camera);

			if (!camera.isOpened())
			{
				std::cerr << "Can't find " + std::to_string(num_of_chosen_camera) + "-th camera";
				return;
			}
		}

		CheckersDetector(std::string path)
		{
			//load video from path
			camera.open(path);

			if (!camera.isOpened())
			{
				std::cerr << "Can't find video";
				return;
			}
		}

		~CheckersDetector()
		{

		}

		ISXFrame::Frame CaptureFrame()
		{
			camera.read(frame.get_frame());
			return frame;
		}
		
		
		void DetectVideo(int interval_in_sec = 1); // interval with which a new frame is taken from the camera
		//void DetectPictures(std::wstring file_path); // path to images
		
		bool FindAndDrawCorners(cv::Mat& img); //function to detect inner corners of chessboard
		void MainCannel(cv::Mat& img); 
		int DrawAndCountCircles(cv::Mat& img, std::vector<cv::Vec3f> circles); //Detect and count circles on the picture
		std::vector<cv::Point> FindContour(cv::Mat img); //function to find contour with the biggest area
		std::vector<cv::Point> RectcontourApprox(std::vector<cv::Point> contour) //find rectangle contour of board method#1
		{
			cv::approxPolyDP(contour, contour, cv::arcLength(contour, true) / 15, true);
			return contour;
		}
		std::vector<cv::Point> RectcontourElipse(std::vector<cv::Point> contour); //find rectangle contour of board method#2
		void DrawLines(std::vector<cv::Point> vec, cv::Mat& img, cv::Scalar color) //Draw a lines from the points
		{
			for (int i = 0; i < vec.size(); i++)
			{
				cv::line(img, vec[i], vec[(i + 1) % 4], color, 1, cv::LINE_AA);
			}
		}
		bool FindCircles(cv::Mat& img, std::vector<cv::Vec3f>& circles);
		cv::Mat GetTransformed(std::vector<cv::Point> rect, cv::Mat& img);
		bool isInRect(cv::Vec4i rect, cv::Point center)
		{
			bool res = false;
			if ((rect[0] < center.x) && (center.x < rect[1]) && (rect[2] < center.y) && (center.y < rect[3]))
			{
				res = true;
			}
			return res;
		}
		void createMatBoard(cv::Mat& img, cv::Mat& board, std::vector<cv::Vec3f> circles);
		cv::Mat AvgBoard(std::list<cv::Mat> boards, float win_proc);
		bool IsContourOK(std::vector<cv::Point> rect);


private:
		cv::VideoCapture camera/*("D:\\Users\\Mariia\\ABTO_cv_ai_course\\Chekers\\Chekers\\Video\\V80821-194057.mp4")*/;
		ISXFrame::Frame frame;
	};
}

