#include "Color/ColorFilter.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <iostream>
#include "Color/ColorRange.h"

	ColorRange colour_filter_low(0, 0, 0, 255, 255, 255);
	ColorRange colour_filter_high(0, 0, 0, 255, 255, 255);

	ColorRange Yellow(20, 78, 20, 37, 255, 255);
	ColorRange All(0, 0, 0, 180, 255, 255);
	ColorRange Red_Low(0, 50, 50, 10, 255, 255);
	ColorRange Red_high(160, 50, 50, 180, 255, 255);
	ColorRange Green(36, 70, 30, 78, 255, 255);
	ColorRange Blue(78, 92, 18, 133, 253, 250);
	ColorRange Black_Low(0, 0, 0, 180, 255, 50);
	ColorRange Black_High(0, 205, 0, 180, 255, 58);
	ColorRange White_low(123, 0, 156, 116, 28, 255);
	ColorRange White_high(123, 0, 142, 180, 29, 255);
	ColorRange Wood(13, 60, 50, 30, 250, 255);

	void ColorFilter::filterOnColor(cv::Mat& input, cv::Mat& output, cv::Mat& mask_Output, ColorFilter::color aColor)
	{
		switch (aColor) {
		case color::ALL:
			colour_filter_low = All;
			colour_filter_high = All;
			break;
		case color::YELLOW:
			colour_filter_low = Yellow;
			colour_filter_high = Yellow;
			break;
		case color::RED:
			colour_filter_low = Red_Low;
			colour_filter_high = Red_high;
			break;
		case color::GREEN:
			colour_filter_low = Green;
			colour_filter_high = Green;
			break;
		case color::BLUE:
			colour_filter_low = Blue;
			colour_filter_high = Blue;
			break;
		case color::BLACK:
			colour_filter_low = Black_Low;
			colour_filter_high = Black_High;
			break;
		case color::WHITE:
			colour_filter_low = White_low;
			colour_filter_high = White_high;
			break;
		case color::WOOD:
			colour_filter_low = Wood;
			colour_filter_high = Wood;
			break;
		default:
			break;
		}

		cv::Mat mask1, mask2, mask_combi, desta, dstb;
		cv::inRange(input, colour_filter_low.low(), colour_filter_low.high(), mask1);
		cv::inRange(input, colour_filter_high.low(), colour_filter_high.high(), mask2);

		mask_Output = mask1 | mask2;

		desta = cv::Scalar::all(0);
		input.copyTo(dstb, mask_Output);
		cv::cvtColor(dstb, output, cv::COLOR_HSV2BGR);

	}


