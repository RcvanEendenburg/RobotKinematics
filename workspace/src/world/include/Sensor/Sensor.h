#ifndef SENSOR_H_
#define SENSOR_H_

#include "SensorException.h"

	class Sensor
	{
	public:
		virtual cv::Mat getFrame()
        {
		    throw SensorException("not a sensor");
        };
	};


#endif