#include <iostream>

#include "CheckersDetector.h"

const std::string ImageFolder = "./Images/";

void main(int, void*)
{
	ISXCheckersDetector::CheckersDetector x(ImageFolder + "Video1.mp4");
	x.DetectVideo();

	system("pause");
}
