#include "dummy_camera.h"
#include <iostream>
#include <string>

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
        std::cout << "Image is not allocated properly\n";
        return false;
    }

    m_dummyImage.copyTo(image);

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

void DummyCamera::SetDummyImage(const char* path)
{
    m_dummyImage = cv::imread(path);
}

unsigned int DummyCamera::GetWidth()
{
    return m_dummyImage.cols;
}

unsigned int DummyCamera::GetHeight()
{
    return m_dummyImage.rows;
}

unsigned char DummyCamera::GetBitDepth()
{
    return m_dummyImage.step[1]*8;
}