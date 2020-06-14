#ifndef STATICIMAGE_H_
#define STATICIMAGE_H_

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <exception>
#include "Sensor.h"


	class StaticImage : public Sensor
	{
	public:
	    explicit StaticImage(std::string path)
	    :path(path)
        {

        }


		cv::Mat getFrame() override
		{
			cv::Mat image = cv::imread(path, CV_LOAD_IMAGE_COLOR);

			if (!image.data)                             
			{
				throw SensorException("Could not open or find the image");
			}
			return image;
		}
	private:
	    std::string path;
	};


#endif