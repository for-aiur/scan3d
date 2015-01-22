#include "scan_calculator.h"
#include <graycode.h>
#include <multiview.h>
#include <iostream>
#include <fstream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

void LoadAbsolutePhase(const char* filename, cv::Mat& absPhase)
{
    cv::Mat dummy = cv::imread(filename, CV_16U);
    short result = 0;

    for(int r = 0; r < absPhase.rows; r++)
    {
        for(int c = 0; c < absPhase.cols; c++)
        {
            unsigned short val = dummy.at<unsigned short>(cv::Point2i(c,r));

            result = val;
            if(val > 32767)
                result = val - 65536;

            absPhase.at<short>(cv::Point2i(c,r)) = result;
        }
    }
}

ScanCalculator::ScanCalculator()
{
}

ScanCalculator::~ScanCalculator()
{

}

bool ScanCalculator::StartCalculation(std::vector<std::vector<cv::Mat> >& sequence, CalibrationResult& c_result)
{
	int height = sequence[0][0].rows;
	int width = sequence[0][0].cols;
    cv::Size size = cv::Size(sequence[0][0].cols, sequence[0][0].rows);
    CalculateAbsPhase(sequence);

	absL = cv::Mat(height, width, CV_16S);
    absR = cv::Mat(height, width, CV_16S);
    LoadAbsolutePhase("absL.tiff", absL);
    LoadAbsolutePhase("absR.tiff", absR);

    std::cout << "K1" << c_result.K[0] << std::endl;
    std::cout << "K2" << c_result.K[1] << std::endl;
    std::cout << "D1" << c_result.D[0] << std::endl;
    std::cout << "D2" << c_result.D[1] << std::endl;
    std::cout << "R" << c_result.R << std::endl;
    std::cout << "T" << c_result.T << std::endl;
    std::cout << "F" << c_result.F << std::endl;
    std::cout << "rvecs1" << c_result.rvecs[0][0] << std::endl;
    std::cout << "rvecs2" << c_result.rvecs[1][0] << std::endl;
    std::cout << "tvecs1" << c_result.tvecs[0][0] << std::endl;
    std::cout << "tvecs2" << c_result.tvecs[1][0] << std::endl;

    cv::stereoRectify(c_result.K[0], c_result.D[0], c_result.K[1], c_result.D[1], size, c_result.R, c_result.T, R1, R2, P1, P2, Q);
    std::cout << "R1" << R1 << std::endl;
    std::cout << "R2" << R2 << std::endl;
    std::cout << "P1" << P1 << std::endl;
    std::cout << "P2" << P2 << std::endl;

    //P = [K*R -(K*R)*C];
    cv::Mat exterior = cv::Mat::zeros(4,4,CV_64F);
    cv::Mat middle = cv::Mat::eye(3,4,CV_64F);
    //std::cout << middle << std::endl;

    PP1 = cv::Mat::zeros(3,4,CV_64F);
    cv::Mat rotmat;
    cv::Rodrigues(c_result.rvecs[0][0], rotmat);
    //rotmat = rotmat.inv();
    exterior.at<double>(0,0) = rotmat.at<double>(0,0);
    exterior.at<double>(0,1) = rotmat.at<double>(0,1);
    exterior.at<double>(0,2) = rotmat.at<double>(0,2);
    exterior.at<double>(1,0) = rotmat.at<double>(1,0);
    exterior.at<double>(1,1) = rotmat.at<double>(1,1);
    exterior.at<double>(1,2) = rotmat.at<double>(1,2);
    exterior.at<double>(2,0) = rotmat.at<double>(2,0);
    exterior.at<double>(2,1) = rotmat.at<double>(2,1);
    exterior.at<double>(2,2) = rotmat.at<double>(2,2);
    exterior.at<double>(0,3) = c_result.tvecs[0][0].at<double>(0);
    exterior.at<double>(1,3) = c_result.tvecs[0][0].at<double>(1);
    exterior.at<double>(2,3) = c_result.tvecs[0][0].at<double>(2);
    exterior.at<double>(3,0) = 0;
    exterior.at<double>(3,1) = 0;
    exterior.at<double>(3,2) = 0;
    exterior.at<double>(3,3) = 1;
    PP1 = c_result.K[0] * middle * exterior;
    PP1 = PP1 / PP1.at<double>(2,3);
    std::cout << "P hesaplanan 1" << PP1 << std::endl;

    PP2 = cv::Mat::zeros(3,4,CV_64F);
    cv::Mat rotmat2;
    cv::Rodrigues(c_result.rvecs[1][0], rotmat2);
    exterior.at<double>(0,0) = rotmat2.at<double>(0,0);
    exterior.at<double>(0,1) = rotmat2.at<double>(0,1);
    exterior.at<double>(0,2) = rotmat2.at<double>(0,2);
    exterior.at<double>(1,0) = rotmat2.at<double>(1,0);
    exterior.at<double>(1,1) = rotmat2.at<double>(1,1);
    exterior.at<double>(1,2) = rotmat2.at<double>(1,2);
    exterior.at<double>(2,0) = rotmat2.at<double>(2,0);
    exterior.at<double>(2,1) = rotmat2.at<double>(2,1);
    exterior.at<double>(2,2) = rotmat2.at<double>(2,2);
    exterior.at<double>(0,3) = c_result.tvecs[1][0].at<double>(0);
    exterior.at<double>(1,3) = c_result.tvecs[1][0].at<double>(1);
    exterior.at<double>(2,3) = c_result.tvecs[1][0].at<double>(2);
    exterior.at<double>(3,0) = 0;
    exterior.at<double>(3,1) = 0;
    exterior.at<double>(3,2) = 0;
    exterior.at<double>(3,3) = 1;
    PP2 = c_result.K[1] * middle * exterior;
    PP2 = PP2 / PP2.at<double>(2,3);
    std::cout << "P hesaplanan 2" << PP2 << std::endl;

    std::vector<cv::Point2f> p;
    std::vector<cv::Point3f> lines;
    for(int r = 1; r < size.height-5; r++)//for(int r = 1; r < size.height-5; r++)
    {
        for(int c = 1; c < 1290; c++)//for(int c = 1; c < size.width-5; c++)
        {
            p.push_back(cv::Point2f(c, r));
        }
    }
    cv::computeCorrespondEpilines(p, 1, c_result.F, lines);
    std::cout << "size of lines: "<< lines.size() << std::endl;

    //3d
    std::vector<cv::Mat> ptCloud;
    m_ptCloud.swap(ptCloud);

    cv::Mat cam0pnts(1,1,CV_64FC2);
    cv::Mat cam1pnts(1,1,CV_64FC2);

    //find correspondence and triangulate
    T2DLINED l;
    int counter = 0;
    for(int r = 1; r < size.height-5; r++)
    {
        std::cout << r << std::endl;
        for(int c = 1; c < 1290; c++)//for(int c = 1; c < size.width-5; c++)
        {
            cv::Mat pnts3D = cv::Mat(1,1,CV_64FC4);
            //boost::chrono::thread_clock::time_point start = boost::chrono::thread_clock::now();

            l.p0.x = 0;//std::max(0,(int)(p[counter].x-150));
            l.p0.y = -(lines[counter].z/lines[counter].y);
            double m = -(lines[counter].x / lines[counter].y);
            double angle = atan(m);            
            l.vr.x = 1250*cos(angle);
            l.vr.y = 1250*sin(angle);

            double _U=0, _V=0;
            double srcW = BLInterpolate(p[counter].x, p[counter].y, absL);
            counter++;

            if(!m_stereoView.DetectPtOnEplipolarLine(l, srcW, absR, _U, _V, 2))
            {
                continue;
            }

            cam0pnts.at<cv::Point2d>(0) = p[counter-1];
            cam1pnts.at<cv::Point2d>(0) = cv::Point2d(_U, _V);

            cv::triangulatePoints(PP1,PP2,cam0pnts,cam1pnts,pnts3D);
            pnts3D = pnts3D / pnts3D.at<double>(3);
            m_ptCloud.push_back(pnts3D);

            //std::cout << pnts3D.at<double>(0) << " " << pnts3D.at<double>(1) << " " << pnts3D.at<double>(2) << std::endl;
            //std::cout << ptCloud[ptCloud.size()-1] << std::endl;

            //boost::chrono::thread_clock::time_point stop = boost::chrono::thread_clock::now();
            //file << "duration: " << boost::chrono::duration_cast<boost::chrono::milliseconds>(stop - start).count() << " ms\n";
        }
    }

    return true;
}

std::vector<cv::Mat>* ScanCalculator::GetCloud()
{
    return& m_ptCloud;
}

void ScanCalculator::CalculateAbsPhase(std::vector<std::vector<cv::Mat> >& sequence)
{
    int height = sequence[0][0].rows;
    int width = sequence[0][0].cols;
    cv::Mat absPhaseL = cv::Mat::zeros( height, width, CV_16U );
    cv::Mat absPhaseR = cv::Mat::zeros( height, width, CV_16U );
    CalculateGP(absPhaseL, sequence[0], 1, 16, 17);
    cv::imwrite("absL.tiff", absPhaseL);
    CalculateGP(absPhaseR, sequence[1], 1, 16, 17);
    cv::imwrite("absR.tiff", absPhaseR);
}
