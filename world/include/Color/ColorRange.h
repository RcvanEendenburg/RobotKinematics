#ifndef COLOURRANGE_H_
#define COLOURRANGE_H_
#include <opencv2/opencv.hpp>


	class ColorRange {
	public:
		ColorRange(int l_hue, int l_sat, int l_val, int h_hue, int h_sat, int h_val)
		:lowRange(cv::Scalar(l_hue,l_sat,l_val)), highRange(cv::Scalar(h_hue, h_sat, h_val))
        {};

		virtual ~ColorRange() = default;

		cv::Scalar low()
		{
			return lowRange;
		}

		cv::Scalar high()
		{
			return highRange;
		}

		std::string toString()
		{
			return std::to_string(lowRange.val[0]) + std::to_string(lowRange.val[1]) + std::to_string(lowRange.val[2])
				+ "|" + std::to_string(highRange.val[0]) + std::to_string(highRange.val[1]) + std::to_string(highRange.val[2]);
		}

	private:
		cv::Scalar lowRange;
		cv::Scalar highRange;
	};

#endif /* COLOURRANGE_H_ */
