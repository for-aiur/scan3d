#include "dummy_camera.h"
#include <iostream>

DummyCamera::DummyCamera()
{

}

DummyCamera::~DummyCamera()
{

}

bool DummyCamera::Open()
{
    std::cout << "Camera has been opened!" << std::endl;
    return true;
}

bool DummyCamera::Close()
{
    std::cout << "Camera has been closed!" << std::endl;
    return true;
}

bool DummyCamera::GetFrame(cv::Mat& image)
{
    if(!image.size().width || !image.size().height)
    {
        std::cout << "Image has not allocated properly\n";
        return false;
    }

    image.setTo(cv::Scalar(255));

    return true;
}

bool DummyCamera::SnapSynchronous()
{
    return true;
}

bool DummyCamera::Wait()
{
    return true;
}

