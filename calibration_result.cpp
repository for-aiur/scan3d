#include "calibration_result.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

CalibrationResult::CalibrationResult()
{
    P[0] = cv::Mat::zeros(3,4,CV_32F);
    P[1] = cv::Mat::zeros(3,4,CV_32F);
    K[0] = cv::Mat::zeros(3,3, CV_64F);
    K[1] = cv::Mat::zeros(3,3, CV_64F);
    D[0] = cv::Mat::zeros(5,1,CV_64F);
    D[1] = cv::Mat::zeros(5,1,CV_64F);
    R = cv::Mat::zeros(3,3, CV_64F);
    F = cv::Mat::zeros(3,3, CV_64F);
    T = cv::Mat::zeros(3,1, CV_64F);
    rvecs[0].resize(1);
    rvecs[0][0] = cv::Mat::zeros(3,1, CV_64F);
    rvecs[1].resize(1);
    rvecs[1][0] = cv::Mat::zeros(3,1, CV_64F);
    tvecs[0].resize(1);
    tvecs[0][0] = cv::Mat::zeros(3,1, CV_64F);
    tvecs[1].resize(1);
    tvecs[1][0] = cv::Mat::zeros(3,1, CV_64F);
    RMS[0] = 0.0;
    RMS[1] = 0.0;
    stereoRMS = 0.0;
}

bool CalibrationResult::SaveIni(const char* filename)const
{
    //Save K[0], K[1], D[0], D[1], R, T

    boost::property_tree::ptree pt;
    pt.put("K0.00", K[0].at<double>(0,0));
    pt.put("K0.01", K[0].at<double>(0,1));
    pt.put("K0.02", K[0].at<double>(0,2));
    pt.put("K0.10", K[0].at<double>(1,0));
    pt.put("K0.11", K[0].at<double>(1,1));
    pt.put("K0.12", K[0].at<double>(1,2));
    pt.put("K0.20", K[0].at<double>(2,0));
    pt.put("K0.21", K[0].at<double>(2,1));
    pt.put("K0.22", K[0].at<double>(2,2));

    pt.put("K1.00", K[1].at<double>(0,0));
    pt.put("K1.01", K[1].at<double>(0,1));
    pt.put("K1.02", K[1].at<double>(0,2));
    pt.put("K1.10", K[1].at<double>(1,0));
    pt.put("K1.11", K[1].at<double>(1,1));
    pt.put("K1.12", K[1].at<double>(1,2));
    pt.put("K1.20", K[1].at<double>(2,0));
    pt.put("K1.21", K[1].at<double>(2,1));
    pt.put("K1.22", K[1].at<double>(2,2));

    pt.put("D0.00", D[0].at<double>(0));
    pt.put("D0.01", D[0].at<double>(1));
    pt.put("D0.02", D[0].at<double>(2));
    pt.put("D0.03", D[0].at<double>(3));
    pt.put("D0.04", D[0].at<double>(4));

    pt.put("D1.00", D[1].at<double>(0));
    pt.put("D1.01", D[1].at<double>(1));
    pt.put("D1.02", D[1].at<double>(2));
    pt.put("D1.03", D[1].at<double>(3));
    pt.put("D1.04", D[1].at<double>(4));

    pt.put("R.00", R.at<double>(0,0));
    pt.put("R.01", R.at<double>(0,1));
    pt.put("R.02", R.at<double>(0,2));
    pt.put("R.10", R.at<double>(1,0));
    pt.put("R.11", R.at<double>(1,1));
    pt.put("R.12", R.at<double>(1,2));
    pt.put("R.20", R.at<double>(2,0));
    pt.put("R.21", R.at<double>(2,1));
    pt.put("R.22", R.at<double>(2,2));

    pt.put("rvecs1.00", rvecs[0][0].at<double>(0));
    pt.put("rvecs1.01", rvecs[0][0].at<double>(1));
    pt.put("rvecs1.02", rvecs[0][0].at<double>(2));

    pt.put("rvecs2.00", rvecs[1][0].at<double>(0));
    pt.put("rvecs2.01", rvecs[1][0].at<double>(1));
    pt.put("rvecs2.02", rvecs[1][0].at<double>(2));

    pt.put("tvecs1.00", tvecs[0][0].at<double>(0));
    pt.put("tvecs1.01", tvecs[0][0].at<double>(1));
    pt.put("tvecs1.02", tvecs[0][0].at<double>(2));

    pt.put("tvecs2.00", tvecs[1][0].at<double>(0));
    pt.put("tvecs2.01", tvecs[1][0].at<double>(1));
    pt.put("tvecs2.02", tvecs[1][0].at<double>(2));

    pt.put("F.00", F.at<double>(0,0));
    pt.put("F.01", F.at<double>(0,1));
    pt.put("F.02", F.at<double>(0,2));
    pt.put("F.10", F.at<double>(1,0));
    pt.put("F.11", F.at<double>(1,1));
    pt.put("F.12", F.at<double>(1,2));
    pt.put("F.20", F.at<double>(2,0));
    pt.put("F.21", F.at<double>(2,1));
    pt.put("F.22", F.at<double>(2,2));

    pt.put("T.00", T.at<double>(0));
    pt.put("T.01", T.at<double>(1));
    pt.put("T.02", T.at<double>(2));

    boost::property_tree::write_ini(filename, pt);

    return true;
}

bool CalibrationResult::LoadIni(const char* filename)
{
    boost::property_tree::ptree pt;
    boost::property_tree::read_ini(filename, pt);

    K[0].at<double>(0,0) = pt.get<double>("K0.00");
    K[0].at<double>(0,1) = pt.get<double>("K0.01");
    K[0].at<double>(0,2) = pt.get<double>("K0.02");
    K[0].at<double>(1,0) = pt.get<double>("K0.10");
    K[0].at<double>(1,1) = pt.get<double>("K0.11");
    K[0].at<double>(1,2) = pt.get<double>("K0.12");
    K[0].at<double>(2,0) = pt.get<double>("K0.20");
    K[0].at<double>(2,1) = pt.get<double>("K0.21");
    K[0].at<double>(2,2) = pt.get<double>("K0.22");

    K[1].at<double>(0,0) = pt.get<double>("K1.00");
    K[1].at<double>(0,1) = pt.get<double>("K1.01");
    K[1].at<double>(0,2) = pt.get<double>("K1.02");
    K[1].at<double>(1,0) = pt.get<double>("K1.10");
    K[1].at<double>(1,1) = pt.get<double>("K1.11");
    K[1].at<double>(1,2) = pt.get<double>("K1.12");
    K[1].at<double>(2,0) = pt.get<double>("K1.20");
    K[1].at<double>(2,1) = pt.get<double>("K1.21");
    K[1].at<double>(2,2) = pt.get<double>("K1.22");

    D[0].at<double>(0) = pt.get<double>("D0.00");
    D[0].at<double>(1) = pt.get<double>("D0.01");
    D[0].at<double>(2) = pt.get<double>("D0.02");
    D[0].at<double>(3) = pt.get<double>("D0.03");
    D[0].at<double>(4) = pt.get<double>("D0.04");

    D[1].at<double>(0) = pt.get<double>("D1.00");
    D[1].at<double>(1) = pt.get<double>("D1.01");
    D[1].at<double>(2) = pt.get<double>("D1.02");
    D[1].at<double>(3) = pt.get<double>("D1.03");
    D[1].at<double>(4) = pt.get<double>("D1.04");

    R.at<double>(0,0) = pt.get<double>("R.00");
    R.at<double>(0,1) = pt.get<double>("R.01");
    R.at<double>(0,2) = pt.get<double>("R.02");
    R.at<double>(1,0) = pt.get<double>("R.10");
    R.at<double>(1,1) = pt.get<double>("R.11");
    R.at<double>(1,2) = pt.get<double>("R.12");
    R.at<double>(2,0) = pt.get<double>("R.20");
    R.at<double>(2,1) = pt.get<double>("R.21");
    R.at<double>(2,2) = pt.get<double>("R.22");

    F.at<double>(0,0) = pt.get<double>("F.00");
    F.at<double>(0,1) = pt.get<double>("F.01");
    F.at<double>(0,2) = pt.get<double>("F.02");
    F.at<double>(1,0) = pt.get<double>("F.10");
    F.at<double>(1,1) = pt.get<double>("F.11");
    F.at<double>(1,2) = pt.get<double>("F.12");
    F.at<double>(2,0) = pt.get<double>("F.20");
    F.at<double>(2,1) = pt.get<double>("F.21");
    F.at<double>(2,2) = pt.get<double>("F.22");

    T.at<double>(0) = pt.get<double>("T.00");
    T.at<double>(1) = pt.get<double>("T.01");
    T.at<double>(2) = pt.get<double>("T.02");

    rvecs[0][0].at<double>(0) = pt.get<double>("rvecs1.00");
    rvecs[0][0].at<double>(1) = pt.get<double>("rvecs1.01");
    rvecs[0][0].at<double>(2) = pt.get<double>("rvecs1.02");

    rvecs[1][0].at<double>(0) = pt.get<double>("rvecs2.00");
    rvecs[1][0].at<double>(1) = pt.get<double>("rvecs2.01");
    rvecs[1][0].at<double>(2) = pt.get<double>("rvecs2.02");

    tvecs[0][0].at<double>(0) = pt.get<double>("tvecs1.00");
    tvecs[0][0].at<double>(1) = pt.get<double>("tvecs1.01");
    tvecs[0][0].at<double>(2) = pt.get<double>("tvecs1.02");

    tvecs[1][0].at<double>(0) = pt.get<double>("tvecs2.00");
    tvecs[1][0].at<double>(1) = pt.get<double>("tvecs2.01");
    tvecs[1][0].at<double>(2) = pt.get<double>("tvecs2.02");

    return true;
}
