#include "CheckersDetector.h"

namespace ISXCheckersDetector
{
	bool CheckersDetector::FindAndDrawCorners(cv::Mat& img)
	{
		cv::Size paternsize(7, 7);
		std::vector<cv::Point2f> corners;
		bool is_found = cv::findChessboardCorners(img, paternsize, corners);
		if (!is_found)
		{
			std::cerr << "Chessboard corners not found\n";
		}
		cv::drawChessboardCorners(img, paternsize, cv::Mat::Mat(corners), is_found);
		return is_found;
	}

	void CheckersDetector::AddLightnessColored(cv::Mat& img, int alpha) // Q alpha
	{
		cv::cvtColor(img, img, CV_BGR2HLS);
		std::vector<cv::Mat> channels(3);
		cv::split(img, channels);
		channels[1] = channels[1] + alpha;
		merge(channels, img);
		cv::cvtColor(img, img, CV_HLS2BGR);
	}

	int CheckersDetector::DrawAndCountCircles(cv::Mat& img, std::vector<cv::Vec3f> circles)
	{
		cv::Mat mask(img.size().height, img.size().width, CV_8UC1, cv::Scalar::all(0));
		cv::Mat mask2;
		for (size_t i = 0; i < circles.size(); i++)
		{
			cv::Vec3i c = circles[i];
			cv::Point center = cv::Point(c[0], c[1]);
			int radius = c[2];
			circle(img, center, radius, cv::Scalar(255, 0, 255), 3, cv::LINE_AA);
			circle(mask, center, radius, cv::Scalar(255, 255, 255), -1, cv::FILLED);
		}
		return cv::connectedComponents(mask, mask2);
	}

	std::vector<cv::Point> CheckersDetector::FindContour(cv::Mat img)
	{
		cv::Mat mask = img.clone();
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
			else { continue; }
		}
		return contours[ind];
	}

	std::vector<cv::Point> CheckersDetector::RectcontourElipse(std::vector<cv::Point> contour)
	{
		cv::Point2f box[4];
		cv::RotatedRect elips = cv::minAreaRect(contour);
		elips.points(box); // you can also use boundingRect() but it is not good
		std::vector<cv::Point> vec;
		for (int i = 0; i < 4; i++)
		{
			vec.push_back(box[i]);
		}
		return vec;
	}


	std::vector<cv::Vec3f> CheckersDetector::FindCircles(cv::Mat& img)
	{
		std::vector<cv::Vec3f> circles;
		cv::Mat mask;
		cv::cvtColor(img, mask, CV_BGR2GRAY);

		cv::GaussianBlur(mask, mask, cv::Size(3, 3), 0);
		cv::Canny(mask, mask, 50, 0, 3, false);

		cv::HoughCircles(mask, circles, CV_HOUGH_GRADIENT, 1, mask.rows / 10, 150, 20, 15, 27);

		return circles;
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
		cv::resize(res, res, cv::Size(440, 440));
		return res;
	}

	void CheckersDetector::createMatBoard(cv::Mat& img, cv::Mat& board, std::vector<cv::Vec3f> circles)
	{
		int height = img.size().height - 30;
		int width = img.size().width - 30;
		int xstep = width / 8;
		int ystep = height / 8;

		cv::Vec4i rect(0, 0, 0, 0);

		float t;

		for (int i = 0; i < 8; i++)
		{
			for (int j = 0; j < 8; j++)
			{
				rect[0] = 15 + i*xstep;
				rect[1] = 15 + (i + 1)*xstep;
				rect[2] = 15 + j*ystep;
				rect[3] = 15 + (j + 1)*ystep;
				for (int c = 0; c < circles.size(); c++)
				{
					if (isInRect(rect, cv::Point(circles[c][0], circles[c][1])))
					{
						t = img.at<cv::Vec3b>(cv::Point(circles[c][0], circles[c][1])).val[0];
						if (t > 150)
						{
							board.at<uchar>(cv::Point(i, j)) = 200;
						}
						else
						{
							board.at<uchar>(cv::Point(i, j)) = 40;
						}
					}
				}
			}
		}
	}

	cv::Mat CheckersDetector::AvgBoard(std::list<cv::Mat> boards)
	{
		int n = boards.size();
		cv::Mat res(cv::Size(8, 8), CV_8UC1, cv::Scalar::all(0));

		for (std::list<cv::Mat>::iterator it = boards.begin(); it!=boards.end();it++)
		{
			res = res + (*it).clone() / n;
		}
		for (int i = 0; i < 8; i++)
		{
			for (int j = 0; j < 8; j++)
			{
				if (res.at<uchar>(cv::Point(i, j)) >= 146)
				{
					res.at<uchar>(cv::Point(i, j)) = 200;
				}
				else if (res.at<uchar>(cv::Point(i, j)) <= 106)
				{
					res.at<uchar>(cv::Point(i, j)) = 40;
				}
				else
				{
					res.at<uchar>(cv::Point(i, j)) = 128;
				}
			}
		}
		return res;
	}

	// void DetectPictures(std::wstring file_path); // path to images

//#define SEC std::chrono::seconds
//#define GET_CURRENT_TIME std::chrono::high_resolution_clock::now()

	void CheckersDetector::DetectVideo(int interval_in_sec)
	{

		std::list<cv::Mat> boards;

		auto current_time = std::chrono::high_resolution_clock::now();
		auto time_of_last_frame = (std::chrono::steady_clock::time_point) std::chrono::seconds(-interval_in_sec);

		if (!camera.isOpened())
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

					cv::Mat chess = CaptureFrame().get_frame();
					cv::Mat chesscontour = chess.clone();
					cv::Mat chesstransformed = chess.clone();
					int count;

					std::vector<cv::Point> contour = FindContour(chesscontour);
					std::vector<cv::Point> rect1 = RectcontourApprox(contour);
					std::vector<cv::Point> rect2 = RectcontourElipse(contour);
					DrawLines(rect1, chesscontour, cv::Scalar(0, 0, 255));
					DrawLines(rect2, chesscontour, cv::Scalar(0, 255, 0));

					chesstransformed = GetTransformed(rect1, chesstransformed);
					std::vector<cv::Vec3f> circles2 = FindCircles(chesstransformed);
					count = DrawAndCountCircles(chesstransformed, circles2);
					std::cerr << count - 1 << " right number of figures on the board" << std::endl;

					cv::Mat board(cv::Size(8, 8), CV_8UC1, cv::Scalar::all(128));
					createMatBoard(chesstransformed, board, circles2);

					if (boards.size() >= 8)
					{
						boards.pop_front();
						boards.push_back(board);
						board = AvgBoard(boards);
					}
					else
					{
						boards.push_back(board.clone());
					}

					cv::resize(board, board, chesstransformed.size(),0,0,cv::INTER_AREA);
					cv::cvtColor(board, board, cv::COLOR_GRAY2BGR);
					cv::Mat output;
					cv::hconcat(chesstransformed, board, output);
					cv::imshow("output", output);
					cv::imshow("chess", chess);

					cv::waitKey(500);
				}
				catch (const std::exception& e)
				{
					//std::cerr << e.what() << '\n';
				}
			}

		}

	}

}