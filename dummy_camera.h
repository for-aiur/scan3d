#include "camera.h"
#include <opencv2/core/core.hpp>

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
};
