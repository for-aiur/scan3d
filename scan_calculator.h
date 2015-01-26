#ifndef SCAN_CALCULATOR_H
#define SCAN_CALCULATOR_H

#include <calibration_result.h>
#include <multiview.h>
#include <opencv2/core/core.hpp>

void LoadAbsolutePhase(const char* filename, cv::Mat& absPhase);

class ScanCalculator{
public:
    ScanCalculator();
    ~ScanCalculator();
    bool StartCalculation(std::vector<std::vector<cv::Mat> >& sequence, CalibrationResult& c_result);

	void CalculateAbsPhase(std::vector<std::vector<cv::Mat> >& sequence);

    std::vector<cv::Mat>* GetCloud();

private:
    cv::Mat absL;
    cv::Mat absR;
    cv::Mat R1, R2, P1, P2, Q, F;
    cv::Mat PP1, PP2;

    std::vector<cv::Mat> m_ptCloud;
	std::vector<std::vector<cv::Mat> > clouds;

    MultiView m_stereoView;

private:
    ScanCalculator(const ScanCalculator& copy);
    ScanCalculator& operator=(const ScanCalculator& rhs);

	void ScanWorker(int threadId, int startRow, int endRow);
};

#endif /* SCAN_CALCULATOR_H */
