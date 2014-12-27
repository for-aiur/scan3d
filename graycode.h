#ifndef GRAYCODE_H
#define GRAYCODE_H
#include <opencv2/core/core.hpp>
#include <vector>

bool CalculateGP(cv::Mat& absPhase, std::vector<cv::Mat> &images, int startGray, int endGray, int startPhase);
bool CalculateGrayCodeImg( cv::Mat& code_img, std::vector<cv::Mat>& images, long StartIndex, long EndIndex );
void EvaluateAbsPhase(cv::Mat& AbsPhase, cv::Mat& Phase, cv::Mat& GCode);
unsigned int BinaryAndOperation(cv::Mat& dst, const cv::Mat& src);
void MaskEvaluation(cv::Mat& Mask, const cv::Mat& Phase1, const cv::Mat& Phase2, const cv::Mat& Phase3, const cv::Mat& Phase4,
                           int DynamicThreshold, int MaximumThreshold, int SinusThreshold );
bool CalculateAbsolutePhase(cv::Mat& phase_img, std::vector<cv::Mat>& images, int offset);
unsigned long LinearCode(unsigned long n);
double BLInterpolate(double x, double y, const cv::Mat &Phase);

#endif /* GRAYCODE_H */
