#include "calibration_result.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/tokenizer.hpp>
#include <numeric.h>

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

void tagTCALPAR::CalcCalibrationParams(const std::vector<TCALPOINT>& ptArray)
{
	double** Mat;

	int rowCount = ptArray.size()*2;
	int colCount = 11;

	double T, U, V, W, X, Y, Z;

	Mat = new double*[rowCount];
	for(int r = 0; r < rowCount; r++)
	{
		Mat[r] = new double[colCount+1];
	}
	
	for( long i = 0; i < ptArray.size(); i++ )
    {
        long j = 2*i;
        X = ptArray[i].X;
        Y = ptArray[i].Y;
        Z = ptArray[i].Z;
        U = ptArray[i].U;
        V = ptArray[i].V;

        Mat[j][ 0] = X;			// dU/dA11 == dU/dL1
        Mat[j][ 1] = Y;			// dU/dA12 == dU/dL2
        Mat[j][ 2] = Z;			// dU/dA13 == dU/dL3
        Mat[j][ 3] = 1.0;		// dU/dA14 == dU/dL4
        Mat[j][ 4] = 0.0;		// dU/dA21 == dU/dL5
        Mat[j][ 5] = 0.0;		// dU/dA22 == dU/dL6
        Mat[j][ 6] = 0.0;		// dU/dA23 == dU/dL7
        Mat[j][ 7] = 0.0;		// dU/dA24 == dU/dL8
        Mat[j][ 8] = -U*X;		// dU/dA41 == dU/dL9
        Mat[j][ 9] = -U*Y;		// dU/dA42 == dU/dL10
        Mat[j][10] = -U*Z;		// dU/dA43 == dU/dL11
        Mat[j][11] = U;			// dU/dA44 == dU/dL12 == 1

        Mat[j+1][ 0] = 0.0;		// dU/dA11 == dU/dL1
        Mat[j+1][ 1] = 0.0;		// dU/dA12 == dU/dL2
        Mat[j+1][ 2] = 0.0;		// dU/dA13 == dU/dL3
        Mat[j+1][ 3] = 0.0;		// dU/dA14 == dU/dL4
        Mat[j+1][ 4] = X;		// dU/dA21 == dU/dL5
        Mat[j+1][ 5] = Y;		// dU/dA22 == dU/dL6
        Mat[j+1][ 6] = Z;		// dU/dA23 == dU/dL7
        Mat[j+1][ 7] = 1.0;		// dU/dA24 == dU/dL8
        Mat[j+1][ 8] = -V*X;	// dU/dA41 == dU/dL9
        Mat[j+1][ 9] = -V*Y;	// dU/dA42 == dU/dL10
        Mat[j+1][10] = -V*Z;	// dU/dA43 == dU/dL11
        Mat[j+1][11] = V;		// dU/dA44 == dU/dL12 == 1
    }

	Householder(Mat, rowCount, colCount);

	    // umspeichern der Koeffizienten
	a11 = Mat[ 0][colCount];
    a12 = Mat[ 1][colCount];
    a13 = Mat[ 2][colCount];
    a14 = Mat[ 3][colCount];
    a21 = Mat[ 4][colCount];
    a22 = Mat[ 5][colCount];
    a23 = Mat[ 6][colCount];
    a24 = Mat[ 7][colCount];
    a41 = Mat[ 8][colCount];
    a42 = Mat[ 9][colCount];
    a43 = Mat[10][colCount];
    a44 = 1.0;

	rowCount = 0;
	for( long i = 0; i < ptArray.size(); i++ )
	{
		X = ptArray[i].X;
		Y = ptArray[i].Y;
		Z = ptArray[i].Z;
		W = ptArray[i].W;

		if( W <= 0)
			continue;

		Mat[rowCount][0] = X;
		Mat[rowCount][1] = Y;
		Mat[rowCount][2] = Z;
		Mat[rowCount][3] = 1.0;
		Mat[rowCount][4] = -W*X;
		Mat[rowCount][5] = -W*Y;
		Mat[rowCount][6] = -W*Z;
		Mat[rowCount][7] = W;
		rowCount++;
	}

	colCount = 7;
	Householder(Mat, rowCount, colCount);

	b11 = Mat[0][colCount];
	b12 = Mat[1][colCount];
	b13 = Mat[2][colCount];
	b14 = Mat[3][colCount];
	b41 = Mat[4][colCount];
	b42 = Mat[5][colCount];
	b43 = Mat[6][colCount];
	b44 = 1.0;

	for (int i=0; i<rowCount; i++)
		delete [] Mat[i];
	delete [] Mat;
}

void FillRawDataFromIni(const char* filename, const char* section_name, std::vector<TCALPOINT>& ptArray)
{
	/*fill calib points*/
	boost::property_tree::ptree pt;
	boost::property_tree::ini_parser::read_ini(filename, pt);
	char sec_name[512];
	std::strcpy(sec_name, section_name);
	std::strcat(sec_name, ".NP");

	int amount_points = pt.get<int>(sec_name);
	char buffer[512];
	typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
	boost::char_separator<char> sep(" ");
	ptArray.resize(amount_points);

	for(int i = 0; i < amount_points; i++)
	{
		sprintf(buffer, "%s.P%.3i", section_name, i);
		std::string line = pt.get<std::string>(buffer);
		tokenizer tokens(line, sep);
		cv::Point3f calib_p;
		tokenizer::iterator tok_iter = tokens.begin();
		ptArray[i].X = ::atof(std::string(*tok_iter++).c_str());
		ptArray[i].Y = ::atof(std::string(*tok_iter++).c_str());
		ptArray[i].Z = ::atof(std::string(*tok_iter++).c_str());
		ptArray[i].U = ::atof(std::string(*tok_iter++).c_str());
		ptArray[i].V = ::atof(std::string(*tok_iter++).c_str());
		ptArray[i].W = ::atof(std::string(*tok_iter).c_str());
	}
}
