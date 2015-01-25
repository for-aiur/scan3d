#ifndef CALIBRATION_RESULT_H
#define CALIBRATION_RESULT_H

#include <vector>
#include <opencv2/core/core.hpp>

#define M_PI 3.14159265359

typedef struct tagTCALPOINT {
  double X;
  double Y;
  double Z;
  double U;
  double V;
  double W;
} TCALPOINT;

void FillRawDataFromIni(const char* filename, const char* section_name, std::vector<TCALPOINT>& ptArray);

typedef struct tagTCALPAR {
    tagTCALPAR()
        : a11(0), a12(0), a13(0), a14(0)
        , a21(0), a22(0), a23(0), a24(0)
        , a41(0), a42(0), a43(0), a44(0)
        , b11(0), b12(0), b13(0), b14(0)
        , b41(0), b42(0), b43(0), b44(0)
    {};

	void CalcCalibrationParams(const std::vector<TCALPOINT>& ptArray);

    double a11,a12,a13,a14;		/**< @brief Kamera-Parameter */
    double a21,a22,a23,a24;		/**< @brief Kamera-Parameter */
    double a41,a42,a43,a44;		/**< @brief Kamera-Parameter */
    double b11,b12,b13,b14;		/**< @brief Projektor-Parameter */
    double b41,b42,b43,b44;		/**< @brief Projektor-Parameter */

} TCALPAR;

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

#endif /* CALIBRATION_RESULT_H */
