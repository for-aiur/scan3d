#include <iostream>
#include <calibration_manager.h>
#include <calibration_result.h>
#include <scan_manager.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

void main()
{
	//std::vector<TCALPOINT> ptArray;
	//FillRawDataFromIni("C:/Users/yildirim/Dropbox/Vision/myOCV/images/hosteo1/RawCal.ini","RawCalParams_VirtualCamera123456002",ptArray);

	std::vector<std::vector<cv::Mat> > sequence;
	sequence.resize(2);
	sequence[0].push_back(cv::imread("C:/Users/yildirim/Dropbox/Vision/myOCV/images/hosteo1/imgs/Img_native_01_00.png"));
	sequence[0].push_back(cv::imread("C:/Users/yildirim/Dropbox/Vision/myOCV/images/hosteo1/imgs/Img_native_01_01.png"));
	sequence[0].push_back(cv::imread("C:/Users/yildirim/Dropbox/Vision/myOCV/images/hosteo1/imgs/Img_native_01_02.png"));
	sequence[0].push_back(cv::imread("C:/Users/yildirim/Dropbox/Vision/myOCV/images/hosteo1/imgs/Img_native_01_03.png"));
	sequence[0].push_back(cv::imread("C:/Users/yildirim/Dropbox/Vision/myOCV/images/hosteo1/imgs/Img_native_01_04.png"));
	sequence[0].push_back(cv::imread("C:/Users/yildirim/Dropbox/Vision/myOCV/images/hosteo1/imgs/Img_native_01_05.png"));
	sequence[0].push_back(cv::imread("C:/Users/yildirim/Dropbox/Vision/myOCV/images/hosteo1/imgs/Img_native_01_06.png"));
	sequence[0].push_back(cv::imread("C:/Users/yildirim/Dropbox/Vision/myOCV/images/hosteo1/imgs/Img_native_01_07.png"));
	sequence[0].push_back(cv::imread("C:/Users/yildirim/Dropbox/Vision/myOCV/images/hosteo1/imgs/Img_native_01_08.png"));
	sequence[0].push_back(cv::imread("C:/Users/yildirim/Dropbox/Vision/myOCV/images/hosteo1/imgs/Img_native_01_09.png"));
	sequence[0].push_back(cv::imread("C:/Users/yildirim/Dropbox/Vision/myOCV/images/hosteo1/imgs/Img_native_01_10.png"));
	sequence[0].push_back(cv::imread("C:/Users/yildirim/Dropbox/Vision/myOCV/images/hosteo1/imgs/Img_native_01_11.png"));
	sequence[0].push_back(cv::imread("C:/Users/yildirim/Dropbox/Vision/myOCV/images/hosteo1/imgs/Img_native_01_12.png"));
	sequence[0].push_back(cv::imread("C:/Users/yildirim/Dropbox/Vision/myOCV/images/hosteo1/imgs/Img_native_01_13.png"));
	sequence[0].push_back(cv::imread("C:/Users/yildirim/Dropbox/Vision/myOCV/images/hosteo1/imgs/Img_native_01_14.png"));
	sequence[0].push_back(cv::imread("C:/Users/yildirim/Dropbox/Vision/myOCV/images/hosteo1/imgs/Img_native_01_15.png"));
	sequence[0].push_back(cv::imread("C:/Users/yildirim/Dropbox/Vision/myOCV/images/hosteo1/imgs/Img_native_01_16.png"));
	sequence[0].push_back(cv::imread("C:/Users/yildirim/Dropbox/Vision/myOCV/images/hosteo1/imgs/Img_native_01_17.png"));
	sequence[0].push_back(cv::imread("C:/Users/yildirim/Dropbox/Vision/myOCV/images/hosteo1/imgs/Img_native_01_18.png"));
	sequence[0].push_back(cv::imread("C:/Users/yildirim/Dropbox/Vision/myOCV/images/hosteo1/imgs/Img_native_01_19.png"));
	sequence[0].push_back(cv::imread("C:/Users/yildirim/Dropbox/Vision/myOCV/images/hosteo1/imgs/Img_native_01_20.png"));
	//sequence[0].push_back(cv::imread("C:/Users/yildirim/Dropbox/Vision/myOCV/images/hosteo1/imgs/Img_native_01_21.png"));

	sequence[1].push_back(cv::imread("C:/Users/yildirim/Dropbox/Vision/myOCV/images/hosteo1/imgs/Img_native_02_00.png"));
	sequence[1].push_back(cv::imread("C:/Users/yildirim/Dropbox/Vision/myOCV/images/hosteo1/imgs/Img_native_02_01.png"));
	sequence[1].push_back(cv::imread("C:/Users/yildirim/Dropbox/Vision/myOCV/images/hosteo1/imgs/Img_native_02_02.png"));
	sequence[1].push_back(cv::imread("C:/Users/yildirim/Dropbox/Vision/myOCV/images/hosteo1/imgs/Img_native_02_03.png"));
	sequence[1].push_back(cv::imread("C:/Users/yildirim/Dropbox/Vision/myOCV/images/hosteo1/imgs/Img_native_02_04.png"));
	sequence[1].push_back(cv::imread("C:/Users/yildirim/Dropbox/Vision/myOCV/images/hosteo1/imgs/Img_native_02_05.png"));
	sequence[1].push_back(cv::imread("C:/Users/yildirim/Dropbox/Vision/myOCV/images/hosteo1/imgs/Img_native_02_06.png"));
	sequence[1].push_back(cv::imread("C:/Users/yildirim/Dropbox/Vision/myOCV/images/hosteo1/imgs/Img_native_02_07.png"));
	sequence[1].push_back(cv::imread("C:/Users/yildirim/Dropbox/Vision/myOCV/images/hosteo1/imgs/Img_native_02_08.png"));
	sequence[1].push_back(cv::imread("C:/Users/yildirim/Dropbox/Vision/myOCV/images/hosteo1/imgs/Img_native_02_09.png"));
	sequence[1].push_back(cv::imread("C:/Users/yildirim/Dropbox/Vision/myOCV/images/hosteo1/imgs/Img_native_02_10.png"));
	sequence[1].push_back(cv::imread("C:/Users/yildirim/Dropbox/Vision/myOCV/images/hosteo1/imgs/Img_native_02_11.png"));
	sequence[1].push_back(cv::imread("C:/Users/yildirim/Dropbox/Vision/myOCV/images/hosteo1/imgs/Img_native_02_12.png"));
	sequence[1].push_back(cv::imread("C:/Users/yildirim/Dropbox/Vision/myOCV/images/hosteo1/imgs/Img_native_02_13.png"));
	sequence[1].push_back(cv::imread("C:/Users/yildirim/Dropbox/Vision/myOCV/images/hosteo1/imgs/Img_native_02_14.png"));
	sequence[1].push_back(cv::imread("C:/Users/yildirim/Dropbox/Vision/myOCV/images/hosteo1/imgs/Img_native_02_15.png"));
	sequence[1].push_back(cv::imread("C:/Users/yildirim/Dropbox/Vision/myOCV/images/hosteo1/imgs/Img_native_02_16.png"));
	sequence[1].push_back(cv::imread("C:/Users/yildirim/Dropbox/Vision/myOCV/images/hosteo1/imgs/Img_native_02_17.png"));
	sequence[1].push_back(cv::imread("C:/Users/yildirim/Dropbox/Vision/myOCV/images/hosteo1/imgs/Img_native_02_18.png"));
	sequence[1].push_back(cv::imread("C:/Users/yildirim/Dropbox/Vision/myOCV/images/hosteo1/imgs/Img_native_02_19.png"));
	sequence[1].push_back(cv::imread("C:/Users/yildirim/Dropbox/Vision/myOCV/images/hosteo1/imgs/Img_native_02_20.png"));
	//sequence[1].push_back(cv::imread("C:/Users/yildirim/Dropbox/Vision/myOCV/images/hosteo1/imgs/Img_native_02_21.png"));
	
	CalibrationManager cm;
	cm.DetectMarkers(sequence[0][0], 0);
	cm.GetCalibrationParameters(1).binary_threshold = 40;
	cm.DetectMarkers(sequence[1][0], 1);

	DetectionResult &drL = cm.GetDetectionResult(0);
	DetectionResult &drR = cm.GetDetectionResult(1);
	cv::Mat P1 = cm.DLT(drL.id_markers_world[0], drL.id_markers_image[0]);
	cv::Mat P2 = cm.DLT(drR.id_markers_world[0], drR.id_markers_image[0]);

	std::cout << "P1: " << P1 << std::endl;
	std::cout << "P2: " << P2 << std::endl;

	cm.MatchMarkers(cm.GetCalibrationDescription(), P1, cm.GetDetectionResult(0), 0, sequence[0][0]);
	cm.MatchMarkers(cm.GetCalibrationDescription(), P2, cm.GetDetectionResult(1), 1, sequence[1][0]);

	cv::namedWindow("show");
	cv::imshow("show", sequence[0][0]);
	cv::waitKey(0);

	cv::namedWindow("show");
	cv::imshow("show", sequence[1][0]);
	cv::waitKey(0);

	cm.StereoCalibrate();
	cm.GetCalibrationResult().SaveIni("calib_result.ini");

	cm.WriteRawCal("rawcal.ini", sequence);
	int a = 100;
}


