#include "scan_manager.h"
#include <stdio.h>
#include <iostream>
#include <dummy_camera.h>
#include <calibration_result.h>

ScanManager::ScanManager()
{
    LoadDummyScan();
}

ScanManager::~ScanManager()
{
    /*
    for(int i = 0; i < m_cams.size(); i++)
    {
        delete m_cams[i];
        m_cams[i] = 0;
    }
    */
}

/*
void ScanManager::AddCamera(ICamera* cam2add, eCameraModel model)
{
    if(m_cams.size() <! MAX_CAM_COUNT)
        return;

    switch(model){
        case CAMERA_MODEL_DUMMY:
        {
            m_cams.push_back(dynamic_cast<DummyCamera*>(cam2add));
            break;
        }
    }
}
*/

void ScanManager::StartScan()
{
    std::vector<std::vector<cv::Mat> > sequence;
    sequence.resize(MAX_CAM_COUNT);

    //fill img sequence
    for(int i = 0; i < 22; i++)
    {
        sequence[0].push_back(m_dummyScanRepo[0][i]);
        sequence[1].push_back(m_dummyScanRepo[1][i]);
    }

    CalibrationResult c_result;
    c_result.LoadIni("calib_result.ini");

    //feed and run scan calculator
    m_scanCalculator.StartCalculation(sequence, c_result);

    //get-keep the result

}

void ScanManager::LoadDummyScan()
{
    //todo: add file exist test
    char filename[256];
    m_dummyScanRepo.resize(MAX_CAM_COUNT);
    for(int j = 0; j < MAX_CAM_COUNT; j++)
    {
        for(int i = 0; i < 22; i++)
        {
            sprintf(filename, "/home/yildirim/Dropbox/Vision/myOCV/images/hosteo1/imgs/Img_native_%.2i_%.2i.png",j+1,i);
            m_dummyScanRepo[j].push_back(cv::imread(filename, CV_8UC1));
            std::cout << filename << std::endl;
        }
    }
}
