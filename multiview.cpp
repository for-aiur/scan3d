#include "multiview.h"
#include <numeric.h>
#include <cmath>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/tokenizer.hpp>

float angleBetween(double pt1x, double pt1y, double pt2x, double pt2y)
{
    float len1 = sqrt(pt1x * pt1x + pt1y * pt1y);
    float len2 = sqrt(pt2x * pt2x + pt2y * pt2y);

    float dot = pt1x * pt2x + pt1y * pt2y;

    float a = dot / (len1 * len2);

    if (a >= 1.0)
        return 0.0;
    else if (a <= -1.0)
        return M_PI;
    else
		return std::acos(a); // 0..PI
}

MultiView::MultiView()
{
    histogram.reserve(100);
    pixPosition.reserve(100);
}

MultiView::~MultiView()
{

}

bool MultiView::DetectPtOnEplipolarLine( T2DLINED &DstEpipolarLine,
										 const double W, cv::Mat& DstAbsPhase,
										 const double U_src, const double V_src,
										 double &U_DstCam,
										 double &V_DstCam,
										 const int DstCamIdx,
										 cv::Mat P_dest
										)
{
    if( !W )
    {
        return false;
    }

	cv::Mat p3d_approx = cv::Mat::ones(4,1,CV_64FC1);
	//limit epipolar line search 

	if(Calc3DPoint(U_src, V_src, W, p3d_approx.at<double>(0), p3d_approx.at<double>(1), p3d_approx.at<double>(2), calParam[0]))
	{
		cv::Mat proj_dest = P_dest*p3d_approx;
		proj_dest = proj_dest / proj_dest.at<double>(2);
		
		//project approx image point on the destination epipolar line
		double alpha = angleBetween(DstEpipolarLine.vr.x, DstEpipolarLine.vr.y, proj_dest.at<double>(0)-DstEpipolarLine.p0.x, proj_dest.at<double>(1)-DstEpipolarLine.p0.y);

		cv::Point2d dest;
		cv::Point2d projection(0,0);
		dest.x = DstEpipolarLine.vr.x / cv::norm(DstEpipolarLine.vr);
		dest.y = DstEpipolarLine.vr.y / cv::norm(DstEpipolarLine.vr);

		double dist = cv::norm(DstEpipolarLine.p0 - cv::Point2d(proj_dest.at<double>(0), proj_dest.at<double>(1)));

		if(alpha!=0)
		{
			projection = DstEpipolarLine.p0 + ( std::cos(alpha) * dist) * dest;
			DstEpipolarLine.p0.x = projection.x - (20.0 * dest.x);
			DstEpipolarLine.p0.y = projection.y - (20.0 * dest.y);

			DstEpipolarLine.vr.x = (projection.x + (20*dest.x)) - (DstEpipolarLine.p0.x);
			DstEpipolarLine.vr.y = (projection.y + (20*dest.y)) - (DstEpipolarLine.p0.y);
		}
	}
	
    // Histogram along the line
    if( !GetIntensityByLine_16Bit( DstAbsPhase, DstEpipolarLine.p0, DstEpipolarLine.p0+DstEpipolarLine.vr, histogram, 0.5, pixPosition ) )
    {
        return false;
    }

    int detectPt_nPixsOnLine = pixPosition.size();
    if( !detectPt_nPixsOnLine )
    {
        return false;
    }

    const double averagePhaseDiff = std::min<double>( 100.0, 2.5 * AveragePhaseDifference( histogram ) );

    int detectPt_i_1;
    double detectPt_Histogr_i, detectPt_Histogr_i_1, detectPt_PhaseDiff, detectPt_InterpolPos;
    cv::Point2d detectPt_Pt;

    for( int detectPt_i=1; detectPt_i<detectPt_nPixsOnLine; detectPt_i++ )
    {
        detectPt_i_1 = detectPt_i-1;
        detectPt_Histogr_i = histogram[detectPt_i];
        detectPt_Histogr_i_1 = histogram[detectPt_i_1];

        if( detectPt_Histogr_i_1<=0 || detectPt_Histogr_i<=0 )
        {
            continue;
        }

        detectPt_PhaseDiff = detectPt_Histogr_i - detectPt_Histogr_i_1;
        if( detectPt_PhaseDiff > averagePhaseDiff )
        {
            continue;
        }

        if( detectPt_Histogr_i_1<=W && detectPt_Histogr_i>=W
            || detectPt_Histogr_i_1>=W && detectPt_Histogr_i<=W )
        {
            if( detectPt_PhaseDiff )
            {
                detectPt_InterpolPos = ( W - detectPt_Histogr_i_1 ) / ( detectPt_PhaseDiff );
                detectPt_Pt = pixPosition[detectPt_i_1] + detectPt_InterpolPos*( pixPosition[detectPt_i]-pixPosition[detectPt_i_1] );
            }
            else
            {
                detectPt_Pt = pixPosition[detectPt_i_1] + 0.5*( pixPosition[detectPt_i]-pixPosition[detectPt_i_1] );
            }
            U_DstCam = detectPt_Pt.x;
            V_DstCam = detectPt_Pt.y;
            return true;
        }
    }
    return false;
}

double MultiView::AveragePhaseDifference( std::vector<double> &lineHistogram )
{
    double averagePhaseDiff = 0;
    std::size_t averagePhaseDiffCount = 0;
    const size_t nPixs = lineHistogram.size();
    for( size_t i=1; i<nPixs; i++ )
    {
        if( lineHistogram[i] <= 0 || lineHistogram[i-1] <= 0 )
            continue;

        averagePhaseDiff += fabs(lineHistogram[i] - lineHistogram[i-1]);
        ++averagePhaseDiffCount;
    }

    if( !averagePhaseDiffCount )
        return DBL_MAX;

    averagePhaseDiff /= static_cast<double>(averagePhaseDiffCount);
    return averagePhaseDiff;
}

bool MultiView::GetIntensityByLine_16Bit( const cv::Mat &Img,
                               const cv::Point2d &p1,
                               const cv::Point2d &p2,
                               std::vector<double> &Histogr,
                               double step,
                               std::vector<cv::Point2d> &PixPosition
                               )
{
    if( Img.type() !=  CV_16S)
    {
        return false;
    }

    int intensityByLine_nWidth = Img.cols;
    int intensityByLine_nHeight = Img.rows;

    if( intensityByLine_nWidth==0 || intensityByLine_nHeight==0 )
    {
        return false;
    }

    cv::Point2f intensityByLine_p1_local = p1;
    cv::Point2f intensityByLine_p2_local = p2;

    // check for image borders -> clip if out of image border
    if( ( p1.x < 0.0 || p1.x >= intensityByLine_nWidth ) || ( p1.y < 0.0 || p1.y >= intensityByLine_nHeight )
        || ( p2.x < 0.0 || p2.x >= intensityByLine_nWidth ) || ( p2.y < 0.0 || p2.y >= intensityByLine_nHeight ) )
    {
        //Clip region unutma
        return false;
    }

    double intensityByLine_LineLength = cv::norm(intensityByLine_p2_local - intensityByLine_p1_local );

    if( intensityByLine_LineLength==0 )
    {
        return false;
    }

    double intensityByLine_nLineLength = static_cast<int>( intensityByLine_LineLength / step );	// immer abgerundet

    // calculate direction of already found line part ( from last two found points)
    cv::Point2f intensityByLine_dir = intensityByLine_p2_local - intensityByLine_p1_local;

    //// go one step in line direction
    //m_IntensityByLine_dir = T2DNormalizeVector( m_IntensityByLine_dir ) * step;
    float n = cv::norm(intensityByLine_dir);
    intensityByLine_dir.x /= n;
    intensityByLine_dir.y /= n;

    PixPosition.resize( intensityByLine_nLineLength );	// capacity wurde bereits im Voraus allockiert
    Histogr.resize( intensityByLine_nLineLength );

    cv::Point2f increase;
    cv::Point2f intensityByLine_pixPos;
    int intensityByLine_u0, intensityByLine_u1, intensityByLine_v0, intensityByLine_v1;
    double intensityByLine_u, intensityByLine_v;
    const short* intensityByLine_pPixel;
    double intensityByLine_ph0, intensityByLine_ph1, intensityByLine_ph2, intensityByLine_ph3;
    double weight_tl, weight_tr, weight_bl, weight_br;

    // for each point of the orthogonal line
    for( int intensityByLine_i=0; intensityByLine_i<intensityByLine_nLineLength; intensityByLine_i++ )
    {
        increase = intensityByLine_dir*static_cast<float>(intensityByLine_i);
        intensityByLine_pixPos = intensityByLine_p1_local + increase;

        intensityByLine_u0 = static_cast<long>(intensityByLine_pixPos.x);
        intensityByLine_u1 = intensityByLine_u0 + 1;
        intensityByLine_v0 = static_cast<long>(intensityByLine_pixPos.y);
        intensityByLine_v1 = intensityByLine_v0 + 1;
        intensityByLine_u  = intensityByLine_pixPos.x - intensityByLine_u0;
        intensityByLine_v  = intensityByLine_pixPos.y - intensityByLine_v0;

        if( intensityByLine_u1 >= intensityByLine_nWidth || intensityByLine_v1 >= intensityByLine_nHeight
            || intensityByLine_v0 < 1 || intensityByLine_v0 >= intensityByLine_nHeight+1)
        {
            Histogr[intensityByLine_i] = -1;
        }
        else
        {
            intensityByLine_pPixel = (const short*)( Img.ptr<short>(intensityByLine_v0 - 1));
			intensityByLine_ph0 = static_cast<double>( intensityByLine_pPixel[intensityByLine_u0] );
            intensityByLine_ph2 = static_cast<double>( intensityByLine_pPixel[intensityByLine_u1] );

            intensityByLine_pPixel = (const short*)( Img.ptr<short>(intensityByLine_v1 - 1) );
            intensityByLine_ph1 = static_cast<double>( intensityByLine_pPixel[intensityByLine_u0] );
            intensityByLine_ph3 = static_cast<double>( intensityByLine_pPixel[intensityByLine_u1] );

            if( intensityByLine_ph0==0 || intensityByLine_ph1==0 || intensityByLine_ph2==0 || intensityByLine_ph3==0 )
            {
                Histogr[intensityByLine_i] = 0;
            }
            else
            {
                //Histogr[intensityByLine_i] = BLInterpolation( intensityByLine_ph0, intensityByLine_ph1, intensityByLine_ph2, intensityByLine_ph3, intensityByLine_u, intensityByLine_v );

                weight_tl = (1.0 - intensityByLine_u) * (intensityByLine_v);
                weight_tr = (intensityByLine_u)       * (intensityByLine_v);
                weight_bl = (1.0 - intensityByLine_u) * (1.0-intensityByLine_v);
                weight_br = (intensityByLine_u)       * (1.0-intensityByLine_v);

                Histogr[intensityByLine_i] = (intensityByLine_ph0*weight_tl)+(intensityByLine_ph2*weight_tr)+(intensityByLine_ph1*weight_bl)+(intensityByLine_ph3*weight_br);
            }
            PixPosition[intensityByLine_i] = intensityByLine_pixPos;
        }
    }

    return true;
}

bool MultiView::Calc3DPoint(double U, double V, double W, double &X, double &Y, double &Z, TCALPAR& param)
{
	if( !W )
		return false;

	double A[3][3];
	double b[3];
	double x[3];
	
	A[0][0] = param.a11-U*param.a41;
	A[0][1] = param.a12-U*param.a42;
	A[0][2] = param.a13-U*param.a43;

	A[1][0] = param.a21-V*param.a41;
	A[1][1] = param.a22-V*param.a42;
	A[1][2] = param.a23-V*param.a43;

	A[2][0] = param.b11-W*param.b41;
	A[2][1] = param.b12-W*param.b42;
	A[2][2] = param.b13-W*param.b43;

	b[0] = U*param.a44-param.a14;
	b[1] = V*param.a44-param.a24;
	b[2] = W*param.b44-param.b14;

	// Berechnung der Koordinaten durch lösen des LGS
	if( !Solve3x3LGS(A, b, x, 1.0E-12) )
		return false;

	// umspeichern der Koordinaten
	X = x[0];
	Y = x[1];
	Z = x[2];

	return true;
}

TCALPAR& MultiView::GetCalParam(int idx)
{
	return calParam[idx];
}

void MultiView::ReadRawCal(const char *filename)
{
	std::vector<TCALPOINT> cam1_calp,cam2_calp;

    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(filename, pt);

    int amount_points = pt.get<int>("Cam1.NP");
    char buffer[32];
    typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
    boost::char_separator<char> sep(" ");

    for(int i = 0; i < amount_points; i++)
    {
        sprintf(buffer, "Cam1.Mark_%.3i", i);
        std::string line = pt.get<std::string>(buffer);
        tokenizer tokens(line, sep);
		TCALPOINT calib_p;
        tokenizer::iterator tok_iter = tokens.begin();
        calib_p.X = ::atof(std::string(*tok_iter++).c_str());
        calib_p.Y = ::atof(std::string(*tok_iter++).c_str());
        calib_p.Z = ::atof(std::string(*tok_iter++).c_str());
		calib_p.U = ::atof(std::string(*tok_iter++).c_str());
        calib_p.V = ::atof(std::string(*tok_iter++).c_str());
		calib_p.W = ::atof(std::string(*tok_iter).c_str());
        cam1_calp.push_back(calib_p);
    }

	amount_points = pt.get<int>("Cam2.NP");
	for(int i = 0; i < amount_points; i++)
    {
        sprintf(buffer, "Cam2.Mark_%.3i", i);
        std::string line = pt.get<std::string>(buffer);
        tokenizer tokens(line, sep);
		TCALPOINT calib_p;
        tokenizer::iterator tok_iter = tokens.begin();
        calib_p.X = ::atof(std::string(*tok_iter++).c_str());
        calib_p.Y = ::atof(std::string(*tok_iter++).c_str());
        calib_p.Z = ::atof(std::string(*tok_iter++).c_str());
		calib_p.U = ::atof(std::string(*tok_iter++).c_str());
        calib_p.V = ::atof(std::string(*tok_iter++).c_str());
		calib_p.W = ::atof(std::string(*tok_iter).c_str());
        cam2_calp.push_back(calib_p);
    }

	calParam[0].CalcCalibrationParams(cam1_calp);
	calParam[1].CalcCalibrationParams(cam2_calp);
}
