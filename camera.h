#ifndef CAMERA_H
#define CAMERA_H

#include <opencv2/core/core.hpp>

enum eCameraModel{
    CAMERA_MODEL_DUMMY
};

class ICamera
{
public:
    ICamera();
    virtual ~ICamera();

    virtual bool Open()=0;
    virtual bool Close()=0;
    virtual bool GetFrame(cv::Mat& image)=0;
    virtual bool SnapSynchronous()=0;
    virtual bool SnapAsynchronous()=0;
    virtual bool Wait()=0;
    virtual unsigned int GetWidth()=0;
    virtual unsigned int GetHeight()=0;
    virtual unsigned char GetBitDepth()=0;
    virtual double GetMinShutter()=0;
    virtual double GetMaxShutter()=0;

    //unimplemented copy constructor and operator=
    ICamera(const ICamera& copy);
    ICamera& operator=(const ICamera& copy);
};
#endif /* CAMERA_H */
