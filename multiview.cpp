#include "multiview.h"
#include <numeric.h>

MultiView::MultiView()
{
    histogram.reserve(650);
    pixPosition.reserve(650);
}

MultiView::~MultiView()
{

}

bool MultiView::DetectPtOnEplipolarLine( const T2DLINED &DstEpipolarLine,
                              const double W, cv::Mat& DstAbsPhase,
                              double &U_DstCam,
                              double &V_DstCam,
                              const int DstCamIdx
                              )
{
    if( !W )
    {
        return false;
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

        // Bilineare Interpolation
        intensityByLine_u0 = static_cast<long>(intensityByLine_pixPos.x);
        intensityByLine_u1 = intensityByLine_u0 + 1;
        intensityByLine_v0 = static_cast<long>(intensityByLine_pixPos.y);
        intensityByLine_v1 = intensityByLine_v0 + 1;
        intensityByLine_u  = intensityByLine_pixPos.x - intensityByLine_u0;
        intensityByLine_v  = intensityByLine_pixPos.y - intensityByLine_v0;

        // check up whether the image coordinate exceeds the image border
        // the bilinear interpolation uses the pixels m_IntensityByLine_u,m_IntensityByLine_v and m_IntensityByLine_u+1,m_IntensityByLine_v+1
        // this could make problems when the calculation reaches the image borders
        if( intensityByLine_u1 >= intensityByLine_nWidth || intensityByLine_v1 >= intensityByLine_nHeight
            || intensityByLine_v0 < 1 || intensityByLine_v0 >= intensityByLine_nHeight+1)
        {
            Histogr[intensityByLine_i] = -1;
        }
        else
        {
            intensityByLine_pPixel = (const short*)( Img.row(intensityByLine_v0 - 1).data );
            intensityByLine_ph0 = static_cast<double>( intensityByLine_pPixel[intensityByLine_u0] );
            intensityByLine_ph2 = static_cast<double>( intensityByLine_pPixel[intensityByLine_u1] );

            intensityByLine_pPixel = (const short*)( Img.row(intensityByLine_v1 - 1).data );
            intensityByLine_ph1 = static_cast<double>( intensityByLine_pPixel[intensityByLine_u0] );
            intensityByLine_ph3 = static_cast<double>( intensityByLine_pPixel[intensityByLine_u1] );

            //qDebug() << m_IntensityByLine_ph0 << "-" << m_IntensityByLine_ph2 << "-" << m_IntensityByLine_ph1 << "-" << m_IntensityByLine_ph3;

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
            //cv::circle(src[1], intensityByLine_pixPos, 1, cv::Scalar(255, 255, 255), 1);
        }
    }

    //cv::circle(src[1], p1, 3, cv::Scalar(255, 0, 255), 5);
    //cv::circle(src[1], p2, 3, cv::Scalar(255, 0, 255), 5);
    //cv::imshow("Test", src[1]);
    //cv::waitKey();
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