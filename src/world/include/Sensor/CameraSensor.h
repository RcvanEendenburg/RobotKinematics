#ifndef CAMERASENSOR_H_
#define CAMERASENSOR_H_

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include "Sensor.h"


	class CameraSensor : public Sensor
	{
	public:
		cv::Mat getFrame() override
		{
			cv::Mat output;
			cv::VideoCapture cap(0);

			for (int x = 0; x < 10; ++x) {//collect 30 frames because the webcam has to initialize
				cap >> output;

				cv::waitKey(30);
			}

			if (!output.data)
			{
				throw SensorException("Could not open a connection to the camera or no frames were received");
			}
			return output;
		}
	};



#endif
