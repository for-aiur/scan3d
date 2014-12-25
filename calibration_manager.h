#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define M_PI 3.14159265359

struct CalibrationParameters{
    unsigned char binary_threshold = 30;
    int kernel_size = 9;
    int id_marker_position_offset = 3;
    int radial_step_amount = 72;
    int marker_search_radius = 500;
};

struct DetectionResult{
    int amount_points = 0;
    int amount_id_markers = 0;
    std::vector< std::vector<cv::Point3f> > id_markers_world;
    std::vector< std::vector<cv::Point2f> > id_markers_image;
    std::vector< std::vector<cv::Point3f> > ring_markers_world;
    std::vector< std::vector<cv::Point2f> > ring_markers_image;
    std::vector< std::vector<cv::Point2f> > all_markers_image;
};

struct CalibrationDescription{
    int amount_points;
    int amount_id_markers = 0;
    std::vector<cv::Point3f> calib_points;
    std::map<int, cv::Point3f> id_markers_pos;
};

struct CalibrationResult{
    cv::Mat P[2];
    cv::Mat K[2];
};

class CalibrationManager{
public:
    CalibrationManager();
    ~CalibrationManager();

    //Detect Id Markers in camera image
    bool DetectMarkers(cv::Mat& camImg, int idxCam);

    //Find corresponding markers in image using approximate P
    bool MatchMarkers(const CalibrationDescription& desc, const cv::Mat& P, DetectionResult& detectionResult, int idxCam, cv::Mat& camImg);

    bool StereoCalibrate();

    //Returns projection matrixusing real-image point correspondences
    cv::Mat DLT(std::vector<cv::Point3f>& ptsObject, std::vector<cv::Point2f>& ptsImage);

    DetectionResult& GetDetectionResult(int idxCam);
    CalibrationParameters& GetCalibrationParameters(int idxCam);
    CalibrationDescription& GetCalibrationDescription();
    CalibrationManager(const CalibrationManager& copy);
private:
    bool ReadCalibrationDescription();
    cv::Mat Calculate2DConditioning(std::vector<cv::Point2f> in);
    cv::Mat Calculate3DConditioning(std::vector<cv::Point3f> in);

    std::vector<CalibrationParameters> m_calibParams;
    CalibrationDescription m_calibDescription;
    CalibrationResult m_stereoResult;
    std::vector<DetectionResult> m_detectionResult;
};
