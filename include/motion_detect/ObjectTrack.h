#pragma once
#ifndef TrackObject
#define TrackObject
#include<opencv2/opencv.hpp>
#include<iostream>
using namespace std;
using namespace cv;
class Blob {
public:
	vector<Point>_contour;
	vector<Point>centerPos;
	Point preditNextCenter;
	Point currentCenter;//first
	Rect currentRect;//first
	double currentArea;//first
	double currentRate;//first
	//bool isStillBeTracked;//是否还在被跟踪
	//int FramesWithoutMatched;
	Blob(vector<Point>con);
	void Predit();
	
};
#endif // !TrackObject


