#include <iostream>

#include "CheckersDetector.h"

const std::string ImageFolder = "./Images/";

void main(int, void*)
{

	ISXCheckersDetector::CheckersDetector x("http://Cam:111@192.168.43.1:2025/video"); //ImageFolder + "Video1.mp4");
	x.DetectVideo(5);

	system("pause");
}
