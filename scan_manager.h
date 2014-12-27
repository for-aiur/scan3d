#include <camera.h>
#include <vector>

#include <scan_calculator.h>

#define MAX_CAM_COUNT 2

class ScanManager
{
public:
    ScanManager();
    ~ScanManager();

    //void AddCamera(ICamera* cam2add, eCameraModel model);
    void StartScan();

private:
    //std::vector<ICamera*> m_cams;
    ScanCalculator m_scanCalculator;

    //Dummy scan data
    //Should be removed in real implementation
    void LoadDummyScan();
    std::vector<std::vector<cv::Mat> > m_dummyScanRepo;

private:
    ScanManager(const ScanManager& copy);
    ScanManager& operator=(const ScanManager& rhs);
};
