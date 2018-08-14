#include <iostream>
#include <string>

#include <opencv2\core.hpp>
#include <opencv2\highgui.hpp>
#include <opencv2\imgproc.hpp>

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2\xfeatures2d.hpp"
#include "opencv2\features2d.hpp"
#include "opencv2\calib3d.hpp"

const std::string ImageFolder = "./Images/";

using namespace std;

//function to detect inner corners of chessboard
bool findAndDrawCorners(cv::Mat& img)
{
	cv::Size paternsize(7, 7);
	vector<cv::Point2f> corners;
	bool isfound = cv::findChessboardCorners(img, paternsize, corners);
	if (!isfound)
	{
		cout << "did not find corners"<<endl;
	}
	cv::drawChessboardCorners(img, paternsize, cv::Mat::Mat(corners), isfound);
	return isfound;
}

//Make colored picure lighter
void AddLightnessColored(cv::Mat& img, int alpha)
{
	cv::cvtColor(img, img, CV_BGR2HLS);
	vector<cv::Mat> channels(3);
	cv::split(img, channels);
	channels[1] = channels[1] + alpha;
	merge(channels, img);
	cv::cvtColor(img, img, CV_HLS2BGR);
}

//Detect and count circles on the picture
int DrawAndCountCircles(cv::Mat& img,vector<cv::Vec3f> circles)
{
	cv::Mat mask(img.size().height, img.size().width, CV_8UC1, cv::Scalar::all(0));
	cv::Mat mask2;
	for (size_t i = 0; i < circles.size(); i++)
	{
		cv::Vec3i c = circles[i];
		cv::Point center = cv::Point(c[0], c[1]);
		cv::circle(img, center, 1, cv::Scalar(0, 100, 100), 3, cv::LINE_AA);
		int radius = c[2];
		circle(img, center, radius, cv::Scalar(255, 0, 255), 3, cv::LINE_AA);
		circle(mask, center, radius, cv::Scalar(255, 255, 255), -1, cv::FILLED);
	}
	return cv::connectedComponents(mask, mask2);
}

//function to find contour with the biggest area
vector<cv::Point> FindContour(cv::Mat img)
{
	cv::Mat mask = img.clone();
	cv::cvtColor(mask, mask, CV_BGR2GRAY);
	mask = mask*1.5;
	cv::Canny(mask, mask, 100, 150, 3, false);
	cv::GaussianBlur(mask, mask, cv::Size(5, 5), 0);
	cv::equalizeHist(mask, mask);

	vector<std::vector<cv::Point>> contours;
	vector<cv::Vec4i> hierarchy;
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

//find rectangle method#1
vector<cv::Point> RectcontourApprox(vector<cv::Point> contour)
{
	cv::approxPolyDP(contour, contour, cv::arcLength(contour, true) / 25, true);
	return contour;
}

//find rectangle method#2
vector<cv::Point> RectcontourElipse(vector<cv::Point> contour)
{
	cv::Point2f box[4];
	cv::RotatedRect elips = cv::minAreaRect(contour);
	elips.points(box); // you can also use boundingRect() but it is not good
	vector<cv::Point> vec;
	for (int i = 0; i < 4; i++)
	{
		vec.push_back(box[i]);
	}
	return vec;
}

//Draw a lines from the points
void DrawLines(vector<cv::Point> vec, cv::Mat& img, cv::Scalar color)
{
	for (int i = 0; i < vec.size(); i++)
	{
		cv::line(img, vec[i], vec[(i + 1) % 4], color, 1, cv::LINE_AA);
	}
}

vector<cv::Vec3f> FindCircles(cv::Mat& img)
{
	vector<cv::Vec3f> circles;
	cv::Mat imggrey;
	cv::cvtColor(img, imggrey, CV_BGR2GRAY);
	cv::Mat mask = imggrey*1.5;

	cv::Canny(mask, mask, 100, 256, 3, false);
	cv::GaussianBlur(mask, mask, cv::Size(5, 5), 0);
	cv::equalizeHist(mask, mask);

	cv::HoughCircles(mask, circles, CV_HOUGH_GRADIENT, 1, mask.rows / 10, 150, 20, 15, 28);

	return circles;
}
void main(int, void*)
{
	cv::Mat chess = cv::imread(ImageFolder + "chess4.jpg");
	cv::resize(chess, chess,cv::Size(440,440));
	cv::Mat chesscontour = chess.clone();
	cv::Mat chesstransformed = chess.clone();

	//findAndDrawCorners(chess);

	vector<cv::Vec3f> circles = FindCircles(chess);

	int count = DrawAndCountCircles(chess,circles);
	cout << count << " figures on the board" << endl;

	vector<cv::Point> contour = FindContour(chesscontour);
	vector<cv::Point> rect1 = RectcontourApprox(contour);
	vector<cv::Point> rect2 = RectcontourElipse(contour);
	DrawLines(rect1, chesscontour, cv::Scalar(0, 0, 255));
	DrawLines(rect2, chesscontour, cv::Scalar(0, 255, 0));

	vector<cv::Point> square;
	square.push_back(cv::Point(0, 0));
	square.push_back(cv::Point((float)std::max(norm(rect1[0] - rect1[1]), norm(rect1[2] - rect1[3])), 0));
	square.push_back(cv::Point((float)std::max(norm(rect1[0] - rect1[1]), norm(rect1[2] - rect1[3])), 
		(float)std::max(norm(rect1[1] - rect1[2]), norm(rect1[3] - rect1[0]))));
	square.push_back(cv::Point(0, (float)std::max(norm(rect1[1] - rect1[2]), norm(rect1[3] - rect1[0]))));

	cv::Mat transform = cv::findHomography(rect1, square);
	cv::Mat res(cv::Size((int)std::max(norm(rect1[0] - rect1[1]), norm(rect1[2] - rect1[3])),
		(int)std::max(norm(rect1[1] - rect1[2]), norm(rect1[3] - rect1[0]))), CV_8UC3);
	cv::warpPerspective(chesstransformed, res, transform, res.size());
	cv::resize(res, res, cv::Size(440, 440));

	vector<cv::Vec3f> circles2 = FindCircles(res);

	cv::Mat drawncircles(res.size(), CV_8UC3, cv::Scalar::all(0));
	DrawAndCountCircles(res, circles2);
	DrawAndCountCircles(drawncircles, circles2);
	cv::Mat transforminv = transform.inv();
	cv::Mat normdrawncircles(cv::Size(440,440),CV_8UC3);
	cv::warpPerspective(drawncircles, normdrawncircles, transforminv, normdrawncircles.size());
	chesstransformed = chesstransformed + normdrawncircles;
	system("pause");
}