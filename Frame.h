#pragma once

#include <opencv2\highgui.hpp>
#include <opencv2\imgproc.hpp>
#include <iostream>


namespace ISXFrame
{
	class Frame
	{
	public:
		Frame() {};
		~Frame() {};

		void SaveFrameToFile(std::string saving_path_and_name)
		{
			if (frame.empty())
			{
				std::cerr << "Frame not valid\n";
			}
			else
			{
				cv::imwrite(saving_path_and_name, frame);
			}
		}

		void ShowFrame()
		{
			if (frame.empty())
			{
				std::cerr << "Frame not valid\n";
			}
			else
			{
				cv::imshow("Output", frame);
			}
		}

		cv::Mat& get_frame()
		{
			if (frame.empty())
			{
				std::cerr << "Frame not valid\n";
			}
			return  frame;
		}

		Frame& operator = (const cv::Mat& new_frame)
		{
			this->frame = new_frame;
			return *this;
		}

		Frame& operator = (const Frame& new_frame)
		{
			this->frame = new_frame.frame;
			return *this;
		}

	private:
		cv::Mat frame;
	};
}
