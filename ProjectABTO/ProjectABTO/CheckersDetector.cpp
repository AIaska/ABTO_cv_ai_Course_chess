#include "stdafx.h"
#include "CheckersDetector.h"

namespace ISXCheckersDetector
{
	bool CheckersDetector::FindAndDrawCorners()
	{
		m_is_board_found = cv::findChessboardCorners(m_chess_transformed.get_frame(), cv::Size(m_board_size - 1, m_board_size - 1), m_corners);
		cv::drawChessboardCorners(m_chess_transformed.get_frame(), cv::Size(m_board_size - 1, m_board_size - 1), cv::Mat::Mat(m_corners), m_is_board_found);
		return m_is_board_found;
	}

	void CheckersDetector::MainChannel(cv::Mat& img)
	{
		std::vector<cv::Mat> channels(3);
		cv::split(img, channels);
		cv::Mat res(img.size(), CV_8UC1, cv::Scalar::all(0));
		cv::Mat i = channels[0];
		res = channels[0] / 3 + channels[1] / 3 + channels[2] / 3;
		channels[0] = res;
		channels[1] = res;
		channels[2] = res;
		merge(channels, img);
	}

	int CheckersDetector::DrawAndCountCircles()
	{
		if (m_circles.size() == 0)
		{
			return 0;
		}

		cv::Mat mask(m_chess_transformed.get_frame().size().height, m_chess_transformed.get_frame().size().width, CV_8UC1, cv::Scalar::all(0));
		cv::Mat mask2;
		cv::Vec3i current_circle;

		int circles_radius = 0;
		for (size_t i = 0; i < m_circles.size(); i++)
		{
			current_circle = m_circles[i];
			circles_radius += current_circle[2];
		}
		circles_radius = circles_radius / m_circles.size();

		for (size_t i = 0; i < m_circles.size(); i++)
		{
			cv::Vec3i c = m_circles[i];
			cv::Point center = cv::Point(c[0], c[1]);
			cv::circle(m_chess_transformed.get_frame(), center, 1, cv::Scalar(0, 100, 100), 3, cv::LINE_AA);
			circle(m_chess_transformed.get_frame(), center, circles_radius, cv::Scalar(255, 0, 255), 3, cv::LINE_AA);
			circle(mask, center, circles_radius, cv::Scalar(255, 255, 255), -1, cv::FILLED);
		}

		return cv::connectedComponents(mask, mask2);
	}

	std::vector<cv::Point> CheckersDetector::FindContour()
	{
		cv::Mat mask = m_chess_contour.get_frame().clone();
		cv::cvtColor(mask, mask, CV_BGR2GRAY);
		cv::Canny(mask, mask, 100, 150, 3, false);
		cv::GaussianBlur(mask, mask, cv::Size(5, 5), 0);
		cv::equalizeHist(mask, mask);

		std::vector<std::vector<cv::Point>> contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::findContours(mask, contours, hierarchy, cv::RetrievalModes::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
		int ind = 0;
		for (int i = 0; i < contours.size(); i++)
		{
			if (cv::contourArea(contours[i]) > cv::contourArea(contours[ind]))
			{
				ind = i;
			}
		}
		m_contour = contours[ind];
		return contours[ind];
	}

	std::vector<cv::Point> CheckersDetector::RectContourElipse()
	{
		cv::Point2f box[4];
		cv::RotatedRect elips = cv::minAreaRect(m_contour);
		elips.points(box);
		std::vector<cv::Point> vec;
		for (int i = 0; i < 4; i++)
		{
			vec.push_back(box[i]);
		}
		return vec;
	}

	int CheckersDetector::FindCircles()
	{
		cv::Mat mask;
		m_chess_transformed.get_frame().copyTo(mask);
		MainChannel(mask);
		cv::cvtColor(mask, mask, CV_BGR2GRAY);
		cv::Mat temp;
		cv::threshold(mask, temp, 80, 256, CV_THRESH_BINARY); //depends on lightness
		//if ((float)cv::countNonZero(temp) / (temp.size().width*temp.size().height) > 0.62)
		//{
		//	return 0;
		//}
		cv::GaussianBlur(mask, mask, cv::Size(3, 3), 0);
		cv::morphologyEx(mask, mask, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));
		cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));

		cv::Canny(mask, mask, 40, 0, 3, false);

		cv::HoughCircles(mask, m_circles, CV_HOUGH_GRADIENT, 1, mask.rows / 10, 150, 20, 15, 27);

		return m_circles.size();;
	}


	cv::Mat CheckersDetector::GetTransformed(std::vector<cv::Point> rect, cv::Mat& img)
	{
		std::vector<cv::Point> square;
		square.push_back(cv::Point(0, 0));
		square.push_back(cv::Point((float)std::max(norm(rect[0] - rect[1]), norm(rect[2] - rect[3])), 0));
		square.push_back(cv::Point((float)std::max(norm(rect[0] - rect[1]), norm(rect[2] - rect[3])),
								(float)std::max(norm(rect[1] - rect[2]), norm(rect[3] - rect[0]))));
		square.push_back(cv::Point(0, (float)std::max(norm(rect[1] - rect[2]), norm(rect[3] - rect[0]))));

		cv::Mat transform = cv::findHomography(rect, square);
		cv::Mat res(cv::Size((int)square[1].x, (int)square[2].y), CV_8UC3);

		cv::warpPerspective(img, res, transform, res.size());
		cv::resize(res, res, cv::Size(m_img_size, m_img_size));
		return res;
	}

	void CheckersDetector::DepictCheckersNoBoard()
	{
		cv::Mat img = m_chess_transformed.get_frame();
		int margin = img.rows * 0.086;
		int cell = (img.rows - (margin * 2)) / m_board_size;
		int radius = cell / 3;
		int step = m_board_depicted.get_frame().cols / m_board_size;

		cv::Vec4i rect(0, 0, 0, 0);
		int pixel_colour = 0;

		for (int i = 0; i < m_board_size; i++)
		{
			for (int j = 0; j < m_board_size; j++)
			{
				rect[0] = margin + i * cell;
				rect[1] = margin + (i + 1) * cell;
				rect[2] = margin + j * cell;
				rect[3] = margin + (j + 1) * cell;
				for (int k = 0; k < m_circles.size(); k++)
				{
					if (IsInRect(rect, cv::Point(m_circles[k][0], m_circles[k][1])))
					{
						for (int l = 0; l < 3; l++)
						{
							pixel_colour += img.at<cv::Vec3b>(cv::Point(m_circles[k][0], m_circles[k][1])).val[l];
						}

						if (pixel_colour > 128 * 3)
						{
							cv::circle(m_board_depicted.get_frame(), cv::Point(i * step + step / 2, j * step + step / 2), radius, cv::Scalar(0xef, 0xd0, 0xa6), cv::FILLED);
						}
						else
						{
							cv::circle(m_board_depicted.get_frame(), cv::Point(i * step + step / 2, j * step + step / 2), radius, cv::Scalar(37, 49, 228), cv::FILLED);
						}
						pixel_colour = 0;
					}
				}
			}
		} 
	}

	bool CheckersDetector::DepictCheckers()
	{
		cv::Mat img = m_chess_transformed.get_frame();
		if (!m_is_board_found)
		{
			DepictCheckersNoBoard();
			return false;
		}

		m_cell_size = ((m_corners[1].x - m_corners[0].x) + (m_corners[m_board_size].y - m_corners[0].y)) / 2; // (width + height) / 2

		if (abs(m_cell_size - 55) > 15)
		{
			m_cell_size = 55;
		}

		cv::Point2f top_left(m_corners[0].x - m_cell_size, m_corners[0].y - m_cell_size);
		cv::Vec4i rect(0, 0, 0, 0);

		float colour;
		int step = m_board_depicted.get_frame().cols / m_board_size;
		int radius = step / 3;

		for (int i = 0; i < m_board_size; i++)
		{
			for (int j = 0; j < m_board_size; j++)
			{
				rect[0] = top_left.x + i * m_cell_size;
				rect[1] = top_left.x + (i + 1) * m_cell_size;
				rect[2] = top_left.y + j * m_cell_size;
				rect[3] = top_left.y + (j + 1) * m_cell_size;
				for (int k = 0; k < m_circles.size(); k++)
				{
					if (IsInRect(rect, cv::Point(m_circles[k][0], m_circles[k][1])))
					{
						colour = img.at<cv::Vec3b>(cv::Point(m_circles[k][0], m_circles[k][1])).val[0];

						if (colour > 150)
						{
							cv::circle(m_board_depicted.get_frame(), cv::Point(i * step + step / 2, j * step + step / 2), radius, cv::Scalar(0xef, 0xd0, 0xa6), cv::FILLED);
						}
						else
						{
							cv::circle(m_board_depicted.get_frame(), cv::Point(i * step + step / 2, j * step + step / 2), radius, cv::Scalar(37, 49, 228), cv::FILLED);
						}
					}
				}
			}
		}
		return true;
	}

	void CheckersDetector::ShowResults(int interval_in_milisec)
	{
		try
		{
			cv::Mat output;
			cv::hconcat(m_chess_transformed.get_frame(), m_board_depicted.get_frame(), output);
			cv::imshow("Checkers detection results", output);
			cv::waitKey(interval_in_milisec);
		}
		catch (const std::exception& e)
		{
			//std::cerr << e.what() << '\n';
		}
	}

	void CheckersDetector::DetectCapturedVideo(std::string video_path_and_name)
	{
		cv::VideoCapture video(video_path_and_name);

		if (!video.isOpened())
		{
			std::cerr << "Video not found\n";
		}

		while (cv::waitKey(5) != 27)
		{
			try
			{
				video >> m_chess.get_frame();
				cv::resize(m_chess.get_frame(), m_chess.get_frame(), cv::Size(m_img_size, m_img_size));
				m_chess_contour = m_chess.get_frame().clone();
				m_chess_transformed = m_chess.get_frame().clone();

				FindContour();
				std::vector<cv::Point> rect1 = RectContourApprox();
				std::vector<cv::Point> rect2 = RectContourElipse();
				DrawLines(rect1, m_chess_contour.get_frame(), cv::Scalar(0, 0, 255));
				DrawLines(rect2, m_chess_contour.get_frame(), cv::Scalar(0, 255, 0));

				m_chess_transformed = GetTransformed(rect1, m_chess_transformed.get_frame());
				cv::rotate(m_chess_transformed.get_frame(), m_chess_transformed.get_frame(), cv::ROTATE_90_COUNTERCLOCKWISE);
				FindCircles();

				m_board_depicted = cv::imread(".\\Images\\empty_board_crop.jpg");
				cv::resize(m_board_depicted.get_frame(), m_board_depicted.get_frame(), cv::Size(m_img_size, m_img_size));

				FindAndDrawCorners();
				DepictCheckers();
				DrawAndCountCircles();
			}
			catch (const std::exception& e)
			{
				//std::cerr << e.what() << '\n';
			}
		ShowResults();
		}
		video.release();
	}

	void CheckersDetector::CaptureAndDetectVideo(int interval_in_sec)
	{
		auto current_time = std::chrono::high_resolution_clock::now();
		auto time_of_last_frame = (std::chrono::steady_clock::time_point) std::chrono::seconds(-interval_in_sec);

		cv::VideoCapture video(0);
		if (!video.isOpened())
		{
			std::cerr << "Camera not found\n";
		}

		while (cv::waitKey(5) != 27)
		{
			current_time = std::chrono::high_resolution_clock::now();
			std::chrono::seconds duration_after_frame = std::chrono::duration_cast<std::chrono::seconds>(current_time - time_of_last_frame);

			if (interval_in_sec <= duration_after_frame.count())
			{
				try
				{
					video.read(m_chess.get_frame());
					cv::resize(m_chess.get_frame(), m_chess.get_frame(), cv::Size(m_img_size, m_img_size));
					m_chess_contour = m_chess.get_frame().clone();
					m_chess_transformed = m_chess.get_frame().clone();

					FindContour();
					std::vector<cv::Point> rect1 = RectContourApprox();
					std::vector<cv::Point> rect2 = RectContourElipse();
					DrawLines(rect1, m_chess_contour.get_frame(), cv::Scalar(0, 0, 255));
					DrawLines(rect2, m_chess_contour.get_frame(), cv::Scalar(0, 255, 0));

					m_chess_transformed = GetTransformed(rect1, m_chess_transformed.get_frame());
					cv::rotate(m_chess_transformed.get_frame(), m_chess_transformed.get_frame(), cv::ROTATE_90_COUNTERCLOCKWISE);
					FindCircles();

					m_board_depicted = cv::imread(".\\Images\\empty_board_crop.jpg");
					cv::resize(m_board_depicted.get_frame(), m_board_depicted.get_frame(), cv::Size(m_img_size, m_img_size));

					FindAndDrawCorners();
					DepictCheckers();
					DrawAndCountCircles();

					cv::waitKey(500);
				}
				catch (const std::exception& e)
				{
					//std::cerr << e.what() << '\n';
				}
			}
			ShowResults();
		}
		video.release();
	}

	void CheckersDetector::DetectPicture(std::string img_path_and_name)
	{
		m_chess = cv::imread(img_path_and_name);
		try
		{
			cv::resize(m_chess.get_frame(), m_chess.get_frame(), cv::Size(m_img_size, m_img_size));
			m_chess_contour = m_chess.get_frame().clone();
			m_chess_transformed = m_chess.get_frame().clone();

			FindContour();
			std::vector<cv::Point> rect1 = RectContourApprox();
			std::vector<cv::Point> rect2 = RectContourElipse();
			DrawLines(rect1, m_chess_contour.get_frame(), cv::Scalar(0, 0, 255));
			DrawLines(rect2, m_chess_contour.get_frame(), cv::Scalar(0, 255, 0));

			m_chess_transformed = GetTransformed(rect1, m_chess_transformed.get_frame());
			cv::rotate(m_chess_transformed.get_frame(), m_chess_transformed.get_frame(), cv::ROTATE_90_COUNTERCLOCKWISE);
			FindCircles();

			m_board_depicted = cv::imread(".\\Images\\empty_board_crop.jpg");
			cv::resize(m_board_depicted.get_frame(), m_board_depicted.get_frame(), cv::Size(m_img_size, m_img_size));

			FindAndDrawCorners();
			DepictCheckers();
			DrawAndCountCircles();
		}
		catch (const std::exception& e)
		{
			//std::cerr << e.what() << '\n';
		}
		ShowResults();
	}
} //ISXCheckersDetector