#include <vector>
#include <opencv2/core/core.hpp>

struct CalibrationResult{

    CalibrationResult();
    CalibrationResult(const CalibrationResult& copy);
    CalibrationResult& operator=(const CalibrationResult& rhs);

    bool SaveIni(const char* filename)const;
    bool LoadIni(const char* filename);

    double RMS[2];
    double stereoRMS;
    cv::Mat P[2];
    cv::Mat K[2];
    cv::Mat D[2];
    cv::Mat R;
    cv::Mat T;
    cv::Mat E;
    cv::Mat F;
    std::vector<cv::Mat> rvecs[2];
    std::vector<cv::Mat> tvecs[2];
};
