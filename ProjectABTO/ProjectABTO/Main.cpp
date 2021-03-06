// Chekers.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <iostream>

#include "CheckersDetector.h"

enum Choice
{
	SAVED_IMAGE = 1, SAVED_VIDEO, CAPTURE_VIDEO, EXIT
};

Choice ChooseImagesSource()
{
	std::cout << "Do you want to...\n";
	std::cout << "1. Process image from a file.\n";
	std::cout << "2. Process video from a file.\n";
	std::cout << "3. Capture your own video.\n";
	std::cout << "4. Exit.\n";

	int answer;
	Choice choice;
	std::cin >> answer;
	choice = static_cast<Choice>(answer);
	return choice;

}

int main()
{
	ISXCheckersDetector::CheckersDetector x;
	//x.CaptureAndDetectVideo();
	Choice choice = ChooseImagesSource();
	bool continue_detection = true;

	while (continue_detection)
	{
		switch (choice)
		{
		case SAVED_IMAGE:
			x.DetectPicture();
			break;
		case SAVED_VIDEO:
			x.DetectCapturedVideo();
			break;
		case CAPTURE_VIDEO:
			x.CaptureAndDetectVideo();
			break;
		case EXIT:
			continue_detection = false;
			break;
		}
	}

	system("pause");
	return 0;
}