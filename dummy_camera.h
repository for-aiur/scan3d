#ifndef DUMMYCAMERA_H
#define DUMMYCAMERA_H

#include <camera.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class DummyCamera : public ICamera
{
public:
    DummyCamera();
    virtual ~DummyCamera();

    virtual bool Open();
    virtual bool Close();
    virtual bool GetFrame(cv::Mat& image);
    virtual bool SnapSynchronous();
    virtual bool SnapAsynchronous();
    virtual bool Wait();
    virtual unsigned int GetWidth();
    virtual unsigned int GetHeight();
    virtual unsigned char GetBitDepth();
    virtual double GetMinShutter();
    virtual double GetMaxShutter();

    void SetDummyImage(const char* path);

private:
    cv::Mat m_dummyImage;
};

#endif /* DUMMYCAMERA_H */
