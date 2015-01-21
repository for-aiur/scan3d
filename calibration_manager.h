#include <calibration_result.h>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define M_PI 3.14159265359

struct CalibrationParameters{
    CalibrationParameters();

    int binary_threshold;
    int kernel_size;
    int id_marker_position_offset;
    int radial_step_amount;
    int marker_search_radius;
};

struct DetectionResult{
    DetectionResult();

    int amount_points;
    int amount_id_markers;
    std::vector< std::vector<cv::Point3f> > id_markers_world;
    std::vector< std::vector<cv::Point2f> > id_markers_image;
    std::vector< std::vector<cv::Point3f> > ring_markers_world;
    std::vector< std::vector<cv::Point2f> > ring_markers_image;
    std::vector< std::vector<cv::Point2f> > all_markers_image;
    cv::Size img_size;
};

struct CalibrationDescription{
    CalibrationDescription();
    void ReadFromIni(const char* filename);

    int amount_points;
    int amount_id_markers;
    std::vector<cv::Point3f> calib_points;
    std::map<int, cv::Point3f> id_markers_pos;
};

class CalibrationManager{
public:
    CalibrationManager();
    ~CalibrationManager();

    //Detect Id Markers in camera image
    bool DetectMarkers(cv::Mat& camImg, const int idxCam);

    //Match markers in calibration description and detected ones
    bool MatchMarkers(const CalibrationDescription& desc, const cv::Mat& P, DetectionResult& detectionResult, const int idxCam, cv::Mat& camImg);

    //Whether enough markers are detected to run stereo calibration or not
    bool IsReadyToGo()const;

    bool StereoCalibrate();

    //Returns projection matrixusing real-image point correspondences
    cv::Mat DLT(const std::vector<cv::Point3f>& ptsObject, const std::vector<cv::Point2f>& ptsImage)const;

    DetectionResult&        GetDetectionResult(const int idxCam);
    CalibrationParameters&  GetCalibrationParameters(const int idxCam);
    CalibrationDescription& GetCalibrationDescription();
    CalibrationResult&      GetCalibrationResult();

    CalibrationManager(const CalibrationManager& copy);
    CalibrationManager& operator=(const CalibrationManager& rhs);

private:
    cv::Mat Calculate2DConditioning(const std::vector<cv::Point2f> in)const;
    cv::Mat Calculate3DConditioning(const std::vector<cv::Point3f> in)const;

    std::vector<CalibrationParameters> m_calibParams;
    CalibrationDescription m_calibDescription;
    CalibrationResult m_stereoResult;
    std::vector<DetectionResult> m_detectionResult;
};
