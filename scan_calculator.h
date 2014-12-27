#ifndef SCAN_CALCULATOR_H
#define SCAN_CALCULATOR_H

#include <opencv2/core/core.hpp>
#include <calibration_result.h>

void LoadAbsolutePhase(const char* filename, cv::Mat& absPhase);

class ScanCalculator{
public:
    ScanCalculator();
    ~ScanCalculator();
    bool StartCalculation(std::vector<std::vector<cv::Mat> >& sequence, CalibrationResult& c_result);

private:
    cv::Mat absL;
    cv::Mat absR;
    cv::Mat R1, R2, P1, P2, Q, F;
    cv::Mat PP1, PP2;

    void CalculateAbsPhase(std::vector<std::vector<cv::Mat> >& sequence);

private:
    ScanCalculator(const ScanCalculator& copy);
    ScanCalculator& operator=(const ScanCalculator& rhs);
};

#endif /* SCAN_CALCULATOR_H */