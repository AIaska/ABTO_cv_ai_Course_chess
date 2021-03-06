#pragma once

#include "opencv2/calib3d.hpp"
#include <chrono>

#include "Frame.h"


namespace ISXCheckersDetector
{
	class CheckersDetector
	{
	public:
		CheckersDetector(int num_of_chosen_camera = 0) : m_board_size(8), m_cell_size(-1), m_img_size(440)
		{

		}

		~CheckersDetector()
		{
			cv::destroyAllWindows();
		}

		void DetectCapturedVideo(std::string video_path_and_name = ".\\Video\\V80821-194057.mp4");
		void CaptureAndDetectVideo(int interval_in_sec = 1 /*interval with which a new frame is taken from the camera*/);
		void DetectPicture(std::string img_path_and_name = ".\\Images\\chess2.jpg");


	private:
		ISXFrame::Frame m_chess;
		ISXFrame::Frame m_chess_contour;
		ISXFrame::Frame m_chess_transformed;
		ISXFrame::Frame m_board_depicted;

		const int m_board_size; // the number of cells in a row / column
		int m_cell_size; // the width / height of one cell
		bool m_is_board_found;
		int m_img_size; //will be resized to this value
		
		std::vector<cv::Point> m_contour;
		std::vector<cv::Point2f> m_corners;
		std::vector<cv::Vec3f> m_circles;
		//int prev_num_of_circles;

	private:
		bool FindAndDrawCorners(); //detect inner corners of chessboard
		void MainChannel(cv::Mat& img);
		int DrawAndCountCircles();
		std::vector<cv::Point> FindContour(); //find contour with the biggest area
		std::vector<cv::Point> RectContourElipse();
		int FindCircles();
		cv::Mat GetTransformed(std::vector<cv::Point> rect, cv::Mat& img);
		bool DepictCheckers(); // returns true if board was found properly, DepictCheckersNoBoard() is called otherwise
		void DepictCheckersNoBoard();
		void ShowResults(int interval_in_milisec = 500 /*wait after showing*/);
		//void CheckIfFrameIsValid(cv::Mat& img);

	private:
		std::vector<cv::Point> RectContourApprox()
		{
			cv::approxPolyDP(m_contour, m_contour, cv::arcLength(m_contour, true) / 25, true);
			return m_contour;
		}

		void DrawLines(std::vector<cv::Point> vec, cv::Mat& img, cv::Scalar color)
		{
			for (int i = 0; i < vec.size(); i++)
			{
				cv::line(img, vec[i], vec[(i + 1) % 4], color, 1, cv::LINE_AA);
			}
		}

		bool IsInRect(cv::Vec4i rect, cv::Point center)
		{
			bool res = false;
			if ((rect[0] < center.x) && (center.x < rect[1]) && (rect[2] < center.y) && (center.y < rect[3]))
			{
				res = true;
			}
			return res;
		}

	};
}