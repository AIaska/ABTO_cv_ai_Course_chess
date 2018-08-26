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

	void CheckersDetector::MainCannel(cv::Mat& img)
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
		std::vector<cv::Point> cont;

		cv::Canny(mask, mask, 100, 150, 3, false);
		cv::GaussianBlur(mask, mask, cv::Size(5, 5), 0);
		cv::equalizeHist(mask, mask);

		std::vector<std::vector<cv::Point>> contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::findContours(mask, contours, hierarchy, cv::RetrievalModes::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
		int ind = NULL;
		for (int i = 0; i < contours.size(); i++)
		{
			if (cv::contourArea(contours[i]) > cv::contourArea(contours[ind]))
			{
				ind = i;
			}
			else { continue; }
		}
		if (ind != NULL)
		{
			cont = contours[ind];
		}
		return cont;
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
		img.copyTo(mask);
		MainCannel(mask);
		cv::cvtColor(mask, mask, CV_BGR2GRAY);
		cv::GaussianBlur(mask, mask, cv::Size(3, 3), 0);
		cv::Canny(mask, mask, 40, 0, 3, false);

		cv::HoughCircles(mask, circles, CV_HOUGH_GRADIENT, 1, mask.rows / 10, 150, 20, 15, 27);

		return circles;
	}

	cv::Mat CheckersDetector::GetTransformed(std::vector<cv::Point> rect, cv::Mat& img)
	{
		std::vector<cv::Point> square;
		std::vector<float> norms;
		std::vector<cv::Point> temp;
		cv::Point p;

		for (int i = 0; i < rect.size(); i++)
		{
			norms.push_back(cv::norm(cv::Mat(rect[i])));
		}
		int min_pos = distance(norms.begin(), min_element(norms.begin(), norms.end()));
		temp.reserve(2 * rect.size());
		temp.insert(temp.end(), rect.begin(), rect.end());
		temp.insert(temp.end(), rect.begin(), rect.end());

		rect[0] = temp[min_pos];
		rect[1] = temp[min_pos + 1];
		rect[2] = temp[min_pos + 2];
		rect[3] = temp[min_pos + 3];
		if (rect[1].y > rect[3].y)
		{
			swap(rect[1], rect[3]);
		}
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

	//remember few old boards and get average of them
	cv::Mat CheckersDetector::AvgBoard(std::list<cv::Mat> boards)
	{
		int n = boards.size();
		cv::Mat res(cv::Size(8, 8), CV_8UC1, cv::Scalar::all(0));

		for (std::list<cv::Mat>::iterator it = boards.begin(); it != boards.end(); it++)
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

	// check if found rect contour is ok
	bool CheckersDetector::IsContourOK(std::vector<cv::Point> rect)
	{
		if (rect.size() != 4)
		{
			return false;
		}
		float diag1 = cv::norm(cv::Mat(rect[0]), cv::Mat(rect[2]));
		float diag2 = cv::norm(cv::Mat(rect[1]), cv::Mat(rect[3]));
		if (std::max(diag1, diag2) / std::min(diag1, diag2) > 1.3)
		{
			return false;
		}
		return true;
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
				//if ()
				//{
				//	std::cout << "The board is empty.";
				//	system("pause");
				//}

				try
				{

					cv::Mat chess = CaptureFrame().get_frame();
					cv::imshow("chess", chess);
					cv::waitKey(100);

					cv::Mat chesscontour = chess.clone();
					cv::Mat chesstransformed = chess.clone();
					int count;

					//FindAndDrawCorners(chess);

					std::vector<cv::Point> contour = FindContour(chesscontour);

					std::vector<cv::Point> rect1 = RectcontourApprox(contour);
					std::vector<cv::Point> rect2 = RectcontourElipse(contour);
					DrawLines(rect1, chesscontour, cv::Scalar(0, 0, 255));
					DrawLines(rect2, chesscontour, cv::Scalar(0, 255, 0));
					cv::imshow("chesscontour", chesscontour);

					if (!IsContourOK(rect2))
					{
						throw std::exception("Found wrong contour");
					}

					chesstransformed = GetTransformed(rect2, chesstransformed);
					std::vector<cv::Vec3f> circles2 = FindCircles(chesstransformed);
					count = DrawAndCountCircles(chesstransformed, circles2);
					std::cout << count - 1 << " right number of figures on the board\n";

					//findAndDrawCorners(chesstransformed);
					cv::Mat board(cv::Size(8, 8), CV_8UC1, cv::Scalar::all(0));
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

					cv::resize(board, board, chesstransformed.size(), 0, 0, cv::INTER_AREA);
					cv::cvtColor(board, board, cv::COLOR_GRAY2BGR);
					cv::Mat output;
					cv::hconcat(chesstransformed, board, output);
					cv::imshow("output", output);

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