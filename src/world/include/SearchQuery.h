#ifndef SEARCHQUERY_H_
#define SEARCHQUERY_H_

#include "Color/ColorFilter.h"
#include "Shape/ShapeFinder.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>


class SearchQuery {
public:
	SearchQuery(cv::Mat& source ,cv::Mat& output,  colourfinder::colour  aColour, shapeFinder::shape aShape);
	virtual ~SearchQuery();

	uint64_t rdtsc();

};




#endif  // SEARCHQUERY_H_

