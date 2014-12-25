#include <opencv2/core/core.hpp>

class Camera{
public:
    Camera();
    virtual ~Camera();

    virtual bool Open()=0;
    virtual bool Close()=0;
    virtual bool GetFrame(cv::Mat& image)=0;
    virtual bool SnapSynchronous()=0;
    virtual bool Wait()=0;
    virtual unsigned int GetWidth()=0;
    virtual unsigned int GetHeight()=0;
    virtual unsigned char GetBitDepth()=0;

    //unimplemented copy constructor
    Camera(const Camera& copy);
};
