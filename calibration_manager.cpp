#include "calibration_manager.h"
#include <boost/lexical_cast.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/tokenizer.hpp>
#include <boost/math/special_functions/round.hpp>

#include <opencv2/flann/miniflann.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <scan_calculator.h>

#include <fstream>
#include <graycode.h>

CalibrationManager::CalibrationManager()
{
    m_calibParams.resize(2);
    m_detectionResult.resize(2);

    m_calibDescription.ReadFromIni("C:/Users/yildirim/Dropbox/Vision/myOCV/images/hosteo1/Calib.ini");
}

CalibrationManager::~CalibrationManager()
{

}

bool CalibrationManager::StereoCalibrate()
{
    if(!IsReadyToGo())
        return false;

    std::vector<std::vector<std::vector<cv::Point2f> > > ring_markers_image_wrapper;
    ring_markers_image_wrapper.resize(2);
    ring_markers_image_wrapper[0].resize(1);
    ring_markers_image_wrapper[1].resize(1);

    for(int cam_idx = 0; cam_idx < 2; cam_idx++)
    {
        //calculate initial guess for Ps
        m_stereoResult.P[cam_idx] = DLT(m_detectionResult[cam_idx].id_markers_world[0], m_detectionResult[cam_idx].id_markers_image[0]);

        //decompose Ps
        cv::decomposeProjectionMatrix(m_stereoResult.P[cam_idx], m_stereoResult.K[cam_idx], m_stereoResult.rvecs[cam_idx][0], m_stereoResult.tvecs[cam_idx][0]);
        m_stereoResult.K[cam_idx] = m_stereoResult.K[cam_idx] / m_stereoResult.K[cam_idx].at<float>(2,2);
        m_stereoResult.K[cam_idx].at<float>(0,1) = 0.0;
        m_stereoResult.K[cam_idx].at<float>(1,0) = 0.0;

        cv::Mat dummy;
        if(!MatchMarkers(m_calibDescription, m_stereoResult.P[cam_idx], m_detectionResult[cam_idx], cam_idx, dummy))
            return false;

        //Tum noktalari buraya dose, bu Levenberg-Marquard kullaniyor guya
        m_stereoResult.RMS[cam_idx] = cv::calibrateCamera(
                                        m_detectionResult[cam_idx].ring_markers_world,
                                        m_detectionResult[cam_idx].ring_markers_image,
                                        m_detectionResult[cam_idx].img_size,
                                        m_stereoResult.K[cam_idx],
                                        m_stereoResult.D[cam_idx],
                                        m_stereoResult.rvecs[cam_idx],
                                        m_stereoResult.tvecs[cam_idx],
                                        CV_CALIB_USE_INTRINSIC_GUESS); //| CV_CALIB_FIX_K1|CV_CALIB_FIX_K2|CV_CALIB_FIX_K3|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);

        cv::projectPoints(m_calibDescription.calib_points,
                          m_stereoResult.rvecs[cam_idx][0],
                          m_stereoResult.tvecs[cam_idx][0],
                          m_stereoResult.K[cam_idx],
                          m_stereoResult.D[cam_idx],
                          ring_markers_image_wrapper[cam_idx][0]);
    }

    //stereo calibration
    std::vector<std::vector<cv::Point3f> > calib_points_wrapper;
    calib_points_wrapper.push_back(m_calibDescription.calib_points);

    m_stereoResult.stereoRMS = cv::stereoCalibrate(calib_points_wrapper,
                                     ring_markers_image_wrapper[0],
                                     ring_markers_image_wrapper[1],
                                     m_stereoResult.K[0],
                                     m_stereoResult.D[0],
                                     m_stereoResult.K[1],
                                     m_stereoResult.D[1],
                                     m_detectionResult[0].img_size,
                                     m_stereoResult.R,
                                     m_stereoResult.T,
                                     m_stereoResult.E,
                                     m_stereoResult.F,
                                     cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 50, 1e-5),
                                     CV_CALIB_USE_INTRINSIC_GUESS );	
    return true;
}

void CalibrationManager::WriteRawCal(const char* filename, std::vector<std::vector<cv::Mat> >& sequence)
{
	int height = sequence[0][0].rows;
	int width = sequence[0][0].cols;

	ScanCalculator sc;
	sc.CalculateAbsPhase(sequence);
	cv::Mat absL = cv::Mat(height, width, CV_16S);
	cv::Mat absR = cv::Mat(height, width, CV_16S);
	LoadAbsolutePhase("absL.tiff", absL);
	LoadAbsolutePhase("absR.tiff", absR);

	int i = 0;
	boost::property_tree::ptree pt;
	char bufferHeader[32];
	char bufferData[256];
	pt.put("Cam1.NP", m_detectionResult[0].ring_markers_image[0].size());
	for(; i < m_detectionResult[0].ring_markers_image[0].size(); i++)
	{
		double imgX = m_detectionResult[0].ring_markers_image[0][i].x;
		double imgY = m_detectionResult[0].ring_markers_image[0][i].y;
		double wrdX = m_detectionResult[0].ring_markers_world[0][i].x;
		double wrdY = m_detectionResult[0].ring_markers_world[0][i].y;
		double wrdZ = m_detectionResult[0].ring_markers_world[0][i].z;
		double srcW = BLInterpolate(imgX, imgY, absL);
		
		sprintf(bufferHeader, "Cam1.Mark_%.3i", i);
		sprintf(bufferData, "%f %f %f %f %f %f", wrdX, wrdY, wrdZ, imgX, imgY, srcW);
		pt.put(bufferHeader, bufferData);
	}

	i = 0;
	pt.put("Cam2.NP", m_detectionResult[1].ring_markers_image[0].size());
	for(; i < m_detectionResult[1].ring_markers_image[0].size(); i++)
	{
		double imgX = m_detectionResult[1].ring_markers_image[0][i].x;
		double imgY = m_detectionResult[1].ring_markers_image[0][i].y;
		double wrdX = m_detectionResult[1].ring_markers_world[0][i].x;
		double wrdY = m_detectionResult[1].ring_markers_world[0][i].y;
		double wrdZ = m_detectionResult[1].ring_markers_world[0][i].z;
		double srcW = BLInterpolate(imgX, imgY, absR);
		
		sprintf(bufferHeader, "Cam2.Mark_%.3i", i);
		sprintf(bufferData, "%f %f %f %f %f %f", wrdX, wrdY, wrdZ, imgX, imgY, srcW);
		pt.put(bufferHeader, bufferData);
	}
	boost::property_tree::write_ini(filename, pt);
}

bool CalibrationManager::DetectMarkers(cv::Mat& camImg, int idxCam)
{
    m_detectionResult[idxCam].amount_id_markers = 0;
    m_detectionResult[idxCam].img_size = cv::Size(camImg.cols, camImg.rows);

    std::map<int, int> id_markers_index;
    std::vector<cv::Point3f> id_markers_world;
    std::vector<cv::Point2f> id_markers_image;
    std::vector<cv::Point2f> all_markers_image;

    //convert source image to gray
    cv::Mat src_gray;
    cv::cvtColor(camImg, src_gray, CV_BGR2GRAY);

    cv::GaussianBlur( src_gray, src_gray, cv::Size(m_calibParams[idxCam].kernel_size, m_calibParams[idxCam].kernel_size), 2, 2 );

    //binarize image
    cv::RNG rng(12345);
    cv::threshold( src_gray, src_gray, m_calibParams[idxCam].binary_threshold, 255, CV_THRESH_BINARY);

    cv::Mat src_binary;
    src_gray.copyTo(src_binary);

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours( src_gray, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

    //Find the rotated rectangles and ellipses for each contour
    std::vector<cv::RotatedRect> minRect( contours.size() );
    std::vector<cv::RotatedRect> minEllipse( contours.size() );

    for( int i = 0; i < contours.size(); i++ )
    {
        minRect[i] = cv::minAreaRect( cv::Mat(contours[i]) );
        if( contours[i].size() > 5 )
        {
            minEllipse[i] = cv::fitEllipse( cv::Mat(contours[i]) );
        }
    }

    //Draw contours + rotated rects + ellipses
    for( int i = 0; i< contours.size(); i++ )
    {
        if(minEllipse[i].size.width < 20 || minEllipse[i].size.width > 100)continue;
        if(minEllipse[i].size.height < 20 || minEllipse[i].size.height > 100)continue;
        if(minEllipse[i].size.area() < 1500 || minEllipse[i].size.area() > 5000)continue;
        if( abs((minEllipse[i].size.width / (double)minEllipse[i].size.height)-1.0) > 0.15 )continue;

        cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        // contour
        //cv::drawContours( camImg, contours, i, cv::Scalar(255,255,0), 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point() );
        // ellipse

        // rotated rectangle
        cv::Point2f rect_points[4]; minRect[i].points( rect_points );
        //for( int j = 0; j < 4; j++ )
          //cv::line( camImg, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );

        //detected_img_points.push_back(minEllipse[i].center);
        //m_detectionResult[idxCam].all_markers_image[0].push_back(minEllipse[i].center);

        cv::Point2f p = cv::Point2f(minEllipse[i].size.width/2-m_calibParams[idxCam].id_marker_position_offset);
        cv::Point2f image_coord = minEllipse[i].center + p;
        int prev_val = src_binary.at<uchar>(image_coord);
        int tooth_counter = 0;

        double step_size = M_PI * 2 / m_calibParams[idxCam].radial_step_amount;
        for(int j = 0; j < m_calibParams[idxCam].radial_step_amount; j++)
        {
            cv::Point2f newp;
            newp.x = (p.x*std::cos(step_size*j) - p.y*std::sin(step_size*j));
            newp.y = (p.x*std::sin(step_size*j) + p.y*std::cos(step_size*j));
            image_coord = minEllipse[i].center + newp;
            //cv::circle( camImg, image_coord, 2,cv::Scalar(255,255,255));

            //analyze this value to detect teeth
            uchar val = src_binary.at<uchar>(image_coord);
            if(abs(val - prev_val) == 255)
                tooth_counter++;
                prev_val = val;
        }

        if(tooth_counter>2){
            m_detectionResult[idxCam].amount_id_markers++;
            id_markers_index[std::ceil(tooth_counter/2.0)] = i;
            cv::circle( camImg, minEllipse[i].center, 20,cv::Scalar(255,0,0), 5);
            std::ostringstream oss;
            oss << std::ceil(tooth_counter/2.0);
            cv::putText(camImg, oss.str(), minEllipse[i].center-cv::Point2f(15,0), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0), 3);
        }
        else
        {
            cv::ellipse( camImg, minEllipse[i], cv::Scalar(0,255,255), 2, 8 );
            all_markers_image.push_back(minEllipse[i].center);
        }
    }

    //find id markers
    for(std::map<int, int>::const_iterator it = id_markers_index.begin(); it != id_markers_index.end();it++)
    {
        id_markers_world.push_back(m_calibDescription.id_markers_pos[(*it).first]);
        id_markers_image.push_back(minEllipse[id_markers_index[(*it).first]].center);
        //std::cout << "id: " << (*it).first << " pos: " << (*it).second << std::endl;
    }
    m_detectionResult[idxCam].id_markers_world[0].swap(id_markers_world);
    m_detectionResult[idxCam].id_markers_image[0].swap(id_markers_image);
    m_detectionResult[idxCam].all_markers_image[0].swap(all_markers_image);

    return true;
}

bool CalibrationManager::MatchMarkers(const CalibrationDescription& desc, const cv::Mat& P, DetectionResult& detectionResult, int idxCam, cv::Mat& camImg)
{
    std::vector<cv::Point3f> ring_markers_world;
    std::vector<cv::Point2f> ring_markers_image;

    cv::flann::KDTreeIndexParams indexParams;
    cv::flann::Index kdtree(cv::Mat(detectionResult.all_markers_image[0]).reshape(1), indexParams);
    for(int i = 0; i < desc.calib_points.size(); i++)
    {
        cv::Mat cp(4,1,CV_32F);
        cp.at<float>(cv::Point2i(0,0)) = desc.calib_points[i].x;
        cp.at<float>(cv::Point2i(0,1)) = desc.calib_points[i].y;
        cp.at<float>(cv::Point2i(0,2)) = desc.calib_points[i].z;
        cp.at<float>(cv::Point2i(0,3)) = 1;

        cv::Mat projected = P*cp;
        projected /= projected.at<float>(2);
        cv::Point2f proj(projected.at<float>(0), projected.at<float>(1));

        cv::circle( camImg, proj, 3,cv::Scalar(0,0,255));
        std::ostringstream oss;
        oss << i;
        cv::putText(camImg, oss.str(), cv::Point2f(proj.x - 25, proj.y), cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(0, 255, 255));

        std::vector<float> query;
        query.push_back(proj.x); //Insert the 2D point we need to find neighbours to the query
        query.push_back(proj.y); //Insert the 2D point we need to find neighbours to the query
        std::vector<int> indices;
        std::vector<float> dists;
        bool result = (bool)kdtree.radiusSearch(query, indices, dists, m_calibParams[idxCam].marker_search_radius, 1);

        if(!result)
            continue;

        //item found
        ring_markers_world.push_back(desc.calib_points[i]);
        ring_markers_image.push_back(detectionResult.all_markers_image[0][indices[0]]);
    }

    m_detectionResult[idxCam].ring_markers_world[0].swap(ring_markers_world);
    m_detectionResult[idxCam].ring_markers_image[0].swap(ring_markers_image);

    return true;
}

DetectionResult& CalibrationManager::GetDetectionResult(const int idx_cam)
{
    return m_detectionResult[idx_cam];
}

CalibrationParameters& CalibrationManager::GetCalibrationParameters(int idxCam)
{
    return m_calibParams[idxCam];
}

CalibrationDescription& CalibrationManager::GetCalibrationDescription()
{
    return m_calibDescription;
}

cv::Mat CalibrationManager::Calculate2DConditioning(std::vector<cv::Point2f> in)const
{
    //translation x ve y lerin ortalamasi t
    cv::Scalar t = cv::mean(in);

    //transform olmus hallerinin abs li ortalamasi s
    for(int i = 0; i < in.size(); i++)
    {
        in[i].x = abs(in[i].x - t[0]);
        in[i].y = abs(in[i].y - t[1]);
    }

    cv::Scalar s = cv::mean(in);

    cv::Mat S = cv::Mat::eye(3,3, CV_32F);
    S.at<float>(cv::Point2i(0,0)) = 1.0/s[0];
    S.at<float>(cv::Point2i(1,1)) = 1.0/s[1];

    cv::Mat Tr = cv::Mat::eye(3,3, CV_32F);
    Tr.at<float>(cv::Point2i(2,0)) = -t[0];
    Tr.at<float>(cv::Point2i(2,1)) = -t[1];

    cv::Mat T;
    T = S*Tr;
    return T;
}

cv::Mat CalibrationManager::Calculate3DConditioning(std::vector<cv::Point3f> in)const
{
    //translation x ve y lerin ortalamasi t
    cv::Scalar t = cv::mean(in);

    //transform olmus hallerinin abs li ortalamasi s
    for(int i = 0; i < in.size(); i++)
    {
        in[i].x = abs(in[i].x - t[0]);
        in[i].y = abs(in[i].y - t[1]);
        in[i].z = abs(in[i].z - t[2]);
    }

    cv::Scalar s = cv::mean(in);

    cv::Mat S = cv::Mat::eye(4,4, CV_32F);
    if(s[0])S.at<float>(cv::Point2i(0,0)) = 1.0/s[0];
    if(s[1])S.at<float>(cv::Point2i(1,1)) = 1.0/s[1];
    if(s[2])S.at<float>(cv::Point2i(2,2)) = 1.0/s[2];

    cv::Mat Tr = cv::Mat::eye(4,4, CV_32F);
    Tr.at<float>(cv::Point2i(3,0)) = -t[0];
    Tr.at<float>(cv::Point2i(3,1)) = -t[1];
    Tr.at<float>(cv::Point2i(3,2)) = -t[2];

    cv::Mat T;
    T = S*Tr;
    return T;
}

cv::Mat CalibrationManager::DLT(const std::vector<cv::Point3f>& ptsObject, const std::vector<cv::Point2f>& ptsImage)const
{
    cv::Mat P = cv::Mat::zeros(3,4,CV_32F);

    if(!ptsObject.size() || !ptsImage.size())
        return P;

    //conditioning
    cv::Mat To = Calculate3DConditioning(ptsObject);
    cv::Mat Ti = Calculate2DConditioning(ptsImage);
    //std::cout <<"To: "<< To << "\n\n";
    //std::cout <<"Ti: "<< Ti << "\n\n";

    std::vector<cv::Mat> conObjPts;
    std::vector<cv::Mat> conImgPts;

    for(int i = 0; i < 6;i++)
    {
        //object points
        cv::Mat om(4,1,CV_32F);
        om.at<float>(cv::Point2i(0,0)) = ptsObject[i].x;
        om.at<float>(cv::Point2i(0,1)) = ptsObject[i].y;
        om.at<float>(cv::Point2i(0,2)) = ptsObject[i].z;
        om.at<float>(cv::Point2i(0,3)) = 1;
        conObjPts.push_back(To*om);
     }

    for(int i = 0; i < 6;i++)
    {
        //image points
        cv::Mat im(3,1,CV_32F);
        im.at<float>(cv::Point2i(0,0)) = ptsImage[i].x;
        im.at<float>(cv::Point2i(0,1)) = ptsImage[i].y;
        im.at<float>(cv::Point2i(0,2)) = 1;
        conImgPts.push_back(Ti*im);
    }

    //Direct Linear Transformation
    //Prepare design(A) matrix
    cv::Mat A(12, 12, CV_32F);
    for(int i = 0; i < conObjPts.size(); i++ )
    {
        A.at<float>(cv::Point2i(0,i*2)) = -conImgPts[i].at<float>(2)*conObjPts[i].at<float>(0);
        A.at<float>(cv::Point2i(1,i*2)) = -conImgPts[i].at<float>(2)*conObjPts[i].at<float>(1);
        A.at<float>(cv::Point2i(2,i*2)) = -conImgPts[i].at<float>(2)*conObjPts[i].at<float>(2);
        A.at<float>(cv::Point2i(3,i*2)) = -conImgPts[i].at<float>(2)*conObjPts[i].at<float>(3);
        A.at<float>(cv::Point2i(4,i*2)) = 0.0;
        A.at<float>(cv::Point2i(5,i*2)) = 0.0;
        A.at<float>(cv::Point2i(6,i*2)) = 0.0;
        A.at<float>(cv::Point2i(7,i*2)) = 0.0;
        A.at<float>(cv::Point2i(8,i*2)) = conImgPts[i].at<float>(0)*conObjPts[i].at<float>(0);
        A.at<float>(cv::Point2i(9,i*2)) = conImgPts[i].at<float>(0)*conObjPts[i].at<float>(1);
        A.at<float>(cv::Point2i(10,i*2)) = conImgPts[i].at<float>(0)*conObjPts[i].at<float>(2);
        A.at<float>(cv::Point2i(11,i*2)) = conImgPts[i].at<float>(0)*conObjPts[i].at<float>(3);

        A.at<float>(cv::Point2i(0,i*2+1)) = 0.0;
        A.at<float>(cv::Point2i(1,i*2+1)) = 0.0;
        A.at<float>(cv::Point2i(2,i*2+1)) = 0.0;
        A.at<float>(cv::Point2i(3,i*2+1)) = 0.0;
        A.at<float>(cv::Point2i(4,i*2+1)) = -conImgPts[i].at<float>(2)*conObjPts[i].at<float>(0);
        A.at<float>(cv::Point2i(5,i*2+1)) = -conImgPts[i].at<float>(2)*conObjPts[i].at<float>(1);
        A.at<float>(cv::Point2i(6,i*2+1)) = -conImgPts[i].at<float>(2)*conObjPts[i].at<float>(2);
        A.at<float>(cv::Point2i(7,i*2+1)) = -conImgPts[i].at<float>(2)*conObjPts[i].at<float>(3);
        A.at<float>(cv::Point2i(8,i*2+1)) = conImgPts[i].at<float>(1)*conObjPts[i].at<float>(0);
        A.at<float>(cv::Point2i(9,i*2+1)) = conImgPts[i].at<float>(1)*conObjPts[i].at<float>(1);
        A.at<float>(cv::Point2i(10,i*2+1)) = conImgPts[i].at<float>(1)*conObjPts[i].at<float>(2);
        A.at<float>(cv::Point2i(11,i*2+1)) = conImgPts[i].at<float>(1)*conObjPts[i].at<float>(3);
    }

    //std::cout << A << std::endl;

    //prepare parameter space
    cv::Mat p(12,1,CV_32F);

    //Solve for 12 parameters
    cv::SVD::solveZ(A, p);

    //Reshape P
    P.at<float>(cv::Point2i(0,0)) = p.at<float>(0);
    P.at<float>(cv::Point2i(1,0)) = p.at<float>(1);
    P.at<float>(cv::Point2i(2,0)) = p.at<float>(2);
    P.at<float>(cv::Point2i(3,0)) = p.at<float>(3);
    P.at<float>(cv::Point2i(0,1)) = p.at<float>(4);
    P.at<float>(cv::Point2i(1,1)) = p.at<float>(5);
    P.at<float>(cv::Point2i(2,1)) = p.at<float>(6);
    P.at<float>(cv::Point2i(3,1)) = p.at<float>(7);
    P.at<float>(cv::Point2i(0,2)) = p.at<float>(8);
    P.at<float>(cv::Point2i(1,2)) = p.at<float>(9);
    P.at<float>(cv::Point2i(2,2)) = p.at<float>(10);
    P.at<float>(cv::Point2i(3,2)) = p.at<float>(11);

    //Reverse condition
    P = Ti.inv()*P*To;
    P = P/P.at<float>(2,3); //make homogenous
    //std::cout << "P " << P << "\n";

    return P;
}

bool CalibrationManager::IsReadyToGo() const
{
    if(m_detectionResult[0].amount_id_markers < 6 || m_detectionResult[1].amount_id_markers < 6 )
        return false;

    if(m_detectionResult[0].ring_markers_world.size() != m_detectionResult[0].ring_markers_image.size())
        return false;

    if(m_detectionResult[1].ring_markers_world.size() != m_detectionResult[1].ring_markers_image.size())
        return false;

    return true;
}

DetectionResult::DetectionResult()
{
    amount_points = 0;
    amount_id_markers = 0;
    img_size = cv::Size(0,0);
    id_markers_world.resize(1);
    id_markers_image.resize(1);
    ring_markers_world.resize(1);
    ring_markers_image.resize(1);
    all_markers_image.resize(1);
}

CalibrationResult& CalibrationManager::GetCalibrationResult()
{
    return m_stereoResult;
}

CalibrationDescription::CalibrationDescription(){
    amount_points = 0;
    amount_id_markers = 0;
}

void CalibrationDescription::ReadFromIni(const char* filename)
{
    //todo: add if file exist condition, this implementation is error prone

    amount_id_markers = 0;
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(filename, pt);

    int amount_points = pt.get<int>("CalibInfo.MarkCount");
    (*this).amount_points = amount_points;
    char buffer[32];
    typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
    boost::char_separator<char> sep(" ");

    for(int i = 0; i < amount_points; i++)
    {
        sprintf(buffer, "Marks.Mark_%.3i", i);
        std::string line = pt.get<std::string>(buffer);
        tokenizer tokens(line, sep);
        cv::Point3f calib_p;
        tokenizer::iterator tok_iter = tokens.begin();
        calib_p.x = ::atof(std::string(*tok_iter++).c_str());
        calib_p.y = ::atof(std::string(*tok_iter++).c_str());
        calib_p.z = ::atof(std::string(*tok_iter).c_str());

        sprintf(buffer, "Marks.Mark_%.3i_ID", i);
        boost::optional<int> id = pt.get_optional<int>(buffer);

        if(id.is_initialized())
        {
            amount_id_markers++;
            id_markers_pos[id.get()] = calib_p;
            continue;
        }

        calib_points.push_back(calib_p);
    }
}

CalibrationParameters::CalibrationParameters()
{
    binary_threshold = 30;
    kernel_size = 9;
    id_marker_position_offset = 3;
    radial_step_amount = 72;
    marker_search_radius = 500;
}