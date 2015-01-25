#ifndef MULTIVIEW_H
#define MULTIVIEW_H
#include <vector>
#include <opencv2/core/core.hpp>
#include <calibration_result.h>

/// A 2D straight line with double precision
struct T2DLINED
{
    cv::Point2d p0;
    cv::Point2d vr;
};

class MultiView{
public:
    MultiView();
    ~MultiView();

    bool DetectPtOnEplipolarLine( T2DLINED &DstEpipolarLine,
                                  const double W, cv::Mat& DstAbsPhase,
								  const double U_src, const double V_src,
                                  double &U_DstCam,
                                  double &V_DstCam,
                                  const int DstCamIdx,
								  cv::Mat P_dest
                                  );

    double AveragePhaseDifference( std::vector<double> &lineHistogram );

    bool GetIntensityByLine_16Bit( const cv::Mat &Img,
                                   const cv::Point2d &p1,
                                   const cv::Point2d &p2,
                                   std::vector<double> &Histogr,
                                   double step,
                                   std::vector<cv::Point2d> &PixPosition
                                   );

	bool Calc3DPoint(double U, double V, double W, double &X, double &Y, double &Z, TCALPAR& param);

	TCALPAR& GetCalParam(int idx);

	void ReadRawCal(const char *filename);

private:
    std::vector<double> histogram;
    std::vector<cv::Point2d> pixPosition;

    MultiView(const MultiView& copy);
    MultiView& operator=(const MultiView& rhs);

	TCALPAR calParam[2];
};

#endif /* MULTIVIEW_H */
