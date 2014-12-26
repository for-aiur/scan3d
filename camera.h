#include <opencv2/core/core.hpp>

class ICamera{
public:
    ICamera();
    virtual ~ICamera();

    virtual bool Open()=0;
    virtual bool Close()=0;
    virtual bool GetFrame(cv::Mat& image)=0;
    virtual bool SnapSynchronous()=0;
    virtual bool Wait()=0;
    virtual unsigned int GetWidth()=0;
    virtual unsigned int GetHeight()=0;
    virtual unsigned char GetBitDepth()=0;

    //unimplemented copy constructor and operator=
    ICamera(const ICamera& copy);
    ICamera& operator=(const ICamera& copy);
};
