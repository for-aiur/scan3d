#ifndef MULTIVIEW_H
#define MULTIVIEW_H
#include <vector>
#include <opencv2/core/core.hpp>

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

    bool DetectPtOnEplipolarLine( const T2DLINED &DstEpipolarLine,
                                  const double W, cv::Mat& DstAbsPhase,
                                  double &U_DstCam,
                                  double &V_DstCam,
                                  const int DstCamIdx
                                  );

    double AveragePhaseDifference( std::vector<double> &lineHistogram );

    bool GetIntensityByLine_16Bit( const cv::Mat &Img,
                                   const cv::Point2d &p1,
                                   const cv::Point2d &p2,
                                   std::vector<double> &Histogr,
                                   double step,
                                   int DstCamIdx,
                                   std::vector<cv::Point2d> &PixPosition
                                   );

private:
    double m_Calc3DPoint_U_SrcCam, m_Calc3DPoint_V_SrcCam;
    std::vector<cv::Point2f> m_EpipolarLine_P;
    std::vector<cv::Point3f> m_EpipolarLine_Pt;
    cv::Point2f m_Calc3DPoint_DstEpipolarLine;

private:
    MultiView(const MultiView& copy);
    MultiView& operator=(const MultiView& rhs);
};

#endif /* MULTIVIEW_H */
