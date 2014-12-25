#include "camera.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class DummyCamera : public Camera
{
public:
    DummyCamera();
    virtual ~DummyCamera();

    virtual bool Open();
    virtual bool Close();
    virtual bool GetFrame(cv::Mat& image);
    virtual bool SnapSynchronous();
    virtual bool Wait();
    virtual unsigned int GetWidth();
    virtual unsigned int GetHeight();
    virtual unsigned char GetBitDepth();

    void SetDummyImage(const char* path);

private:
    cv::Mat m_dummyImage;
};
