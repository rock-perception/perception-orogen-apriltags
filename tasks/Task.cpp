/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace apriltags;

Task::Task(std::string const& name)
    : TaskBase(name), out_frame_ptr(new base::samples::frame::Frame())
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine), out_frame_ptr(new base::samples::frame::Frame())
{
}

Task::~Task()
{
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    // setting camera matrix and distortion matrix
    frame_helper::CameraCalibration cal = _camera_calibration.get();
    camera_k = (cv::Mat_<double>(3,3) << cal.fx, 0, cal.cx, 0, cal.fy, cal.cy, 0, 0, 1);
    camera_dist = (cv::Mat_<double>(1,4) << cal.d0, cal.d1, cal.d2, cal.d3);

    // setting immutable parameters
    tag_code = _tag_code.get();


    rvec.create(3,1,CV_32FC1);
    tvec.create(3,1,CV_32FC1);

    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();

    RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> current_frame_ptr;
    //main loop for detection in each input frame
    for(int count=0; _image.read(current_frame_ptr) == RTT::NewData; ++count)
    {
    	//convert base::samples::frame::Frame to grayscale cv::Mat
    	cv::Mat image;
    	base::samples::frame::Frame temp_frame;

    	if (current_frame_ptr->isBayer())
    	{
    		frame_helper::FrameHelper::convertBayerToRGB24(*current_frame_ptr, temp_frame);
    		image = frame_helper::FrameHelper::convertToCvMat(temp_frame);
    		cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
    	}
    	else
    		image = frame_helper::FrameHelper::convertToCvMat(*current_frame_ptr);

    	// convert to grayscale and undistort both images
    	cv::Mat image_gray;
    	if(image.channels() == 3)
    	{
    		cv::cvtColor(image, image_gray, cv::COLOR_RGB2GRAY);
    		cv::Mat temp;
    		cv::undistort(image_gray, temp, camera_k, camera_dist);
    		image_gray = temp;
    		cv::Mat temp2;
    		cv::undistort(image, temp2, camera_k, camera_dist);
    		image = temp2;
    	}
    	else
    	{
    		cv::undistort(image, image_gray, camera_k, camera_dist);
    		cv::Mat temp;
    		cv::undistort(image, temp, camera_k, camera_dist);
    		image = temp;
    	}


    	//TODO: Create a enum with the tag_codes;

    	//set the apriltag family
    	apriltag_family_t *tf = NULL;
    	(tag_code == "36h11") ? tf = tag36h11_create() : throw std::runtime_error("The desired apriltag family code is not implemented");

    	//TODO: Create orogen properties to set all these values.
    	// create a apriltag detector and add the current family
    	apriltag_detector_t *td = apriltag_detector_create();
    	apriltag_detector_add_family(td, tf);
    	td->quad_decimate = 1.0;
    	td->quad_sigma = 0.0;
    	td->nthreads = 4;
    	td->debug = 0;
    	td->refine_decode = 0;
    	td->refine_pose = 0;

    	//Repeat processing on input set this many
    	int maxiters = 1;
    	const int hamm_hist_max = 10;

    	//main loop
    	for (int iter = 0; iter < maxiters; iter++)
    	{
    		if (maxiters > 1) printf("iter %d / %d\n", iter + 1, maxiters);
    		int hamm_hist[hamm_hist_max];
    		memset(hamm_hist, 0, sizeof(hamm_hist));

    		//convert cv::Mat to image_u8_t
    		image_u8_t *im = image_u8_create(image_gray.cols, image_gray.rows);
    		memcpy(im->buf, image_gray.data, image_gray.total()*image_gray.elemSize());

    		//initialize timer
    		double t0, dt;
    		if(_print_time.get()) t0 = tic();

    		//detect markers
    		zarray_t *detections = apriltag_detector_detect(td, im);
//    		if (zarray_size(detections)) std::cout << "//////// APRILTAGS /////////" << std::endl;

            if(_print_time.get()) dt = tic()-t0;

    		//build the rbs_vector and draw detected markers
    		std::vector<base::samples::RigidBodyState> rbs_vector;
    		for (int i = 0; i < zarray_size(detections); i++)
    		{
    			apriltag_detection_t *det;
    			zarray_get(detections, i, &det);

    			//estimate pose and push_back to rbs_vector
    			base::samples::RigidBodyState rbs;
    			getRbs(rbs, _marker_size.get(), det->p, camera_k, cv::Mat());
    			rbs.sourceFrame = _source_frame.get();
    			rbs.time = current_frame_ptr->time;
    			rbs_vector.push_back(rbs);

    			std::cout << "APRILTAGS ID: " << det->id <<
    					" x: " << rbs.position.x() <<
						" y: " << rbs.position.y() <<
						" z: " << rbs.position.z() <<
						" roll: "  << rbs.getRoll()*180/M_PI  <<
						" pitch: " << rbs.getPitch()*180/M_PI <<
						" yaw: "   << rbs.getYaw()*180/M_PI   <<
						" extract_time: " << dt*1000 << " ms" <<
						" pixel_size: " << det->p[1][1] - det->p[0][1] << std::endl;

    			std::cout << " extract_time: " << dt*1000 << " ms" << std::endl;

    			if (!_quiet.get())
    				printf("detection %3d: id (%2dx%2d)-%-4d, hamming %d, goodness %8.3f, margin %8.3f\n",
    						i, det->family->d*det->family->d, det->family->h, det->id, det->hamming, det->goodness, det->decision_margin);

    			if (_draw_image.get())
    			{
    				draw(image,det->p, det->c, det->id, cv::Scalar(0,0,255), 2);
    				draw3dAxis(image, _marker_size.get(), camera_k, cv::Mat());
    				draw3dCube(image, _marker_size.get(), camera_k, cv::Mat());
    			}

    			hamm_hist[det->hamming]++;

    			apriltag_detection_destroy(det);
    		}
//    		if (zarray_size(detections)) std::cout << "////////////////////////////" << std::endl;
    		zarray_destroy(detections);

    		if (!_quiet.get())
    		{
    			timeprofile_display(td->tp);
    			printf("nedges: %d, nsegments: %d, nquads: %d\n", td->nedges, td->nsegments, td->nquads);
    			printf("Hamming histogram: ");
    			for (int i = 0; i < hamm_hist_max; i++)
    				printf("%5d", hamm_hist[i]);
    			printf("%12.3f", timeprofile_total_utime(td->tp) / 1.0E3);
    			printf("\n");
    		}

    		image_u8_destroy(im);

    		//write the the image in the output port
    		if (_draw_image.get())
    		{
    			base::samples::frame::Frame *pframe = out_frame_ptr.write_access();
    			frame_helper::FrameHelper::copyMatToFrame(image,*pframe);
    			pframe->time = base::Time::now();
    			out_frame_ptr.reset(pframe);
    			_output_image.write(out_frame_ptr);
    		}

    		//write the markers in the output port
    		if (rbs_vector.size() != 0)
    		{
    			rbs_vector.clear();
    			_marker_poses.write(rbs_vector);
    		}
    	}


    	apriltag_detector_destroy(td);
    	tag36h11_destroy(tf);
    }
}

void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}

void Task::getRbs(base::samples::RigidBodyState &rbs, float markerSizeMeters, double points[][2], cv::Mat  camMatrix,cv::Mat distCoeff)throw(cv::Exception)
{
    if (markerSizeMeters<=0)throw cv::Exception(9004,"markerSize<=0: invalid markerSize","getRbs",__FILE__,__LINE__);
    if (camMatrix.rows==0 || camMatrix.cols==0) throw cv::Exception(9004,"CameraMatrix is empty","getRbs",__FILE__,__LINE__);

    double halfSize=markerSizeMeters/2.;

    //set objects points clockwise starting from (-1,+1,0)
    cv::Mat ObjPoints(4,3,CV_32FC1);
    ObjPoints.at<float>(0,0)=-halfSize;
    ObjPoints.at<float>(0,1)=+halfSize;
    ObjPoints.at<float>(0,2)=0;
    ObjPoints.at<float>(1,0)=+halfSize;
    ObjPoints.at<float>(1,1)=+halfSize;
    ObjPoints.at<float>(1,2)=0;
    ObjPoints.at<float>(2,0)=+halfSize;
    ObjPoints.at<float>(2,1)=-halfSize;
    ObjPoints.at<float>(2,2)=0;
    ObjPoints.at<float>(3,0)=-halfSize;
    ObjPoints.at<float>(3,1)=-halfSize;
    ObjPoints.at<float>(3,2)=0;


    //set image points from the detected points
    cv::Mat ImagePoints(4,2,CV_32FC1);
    for (int c=0;c<4;c++)
    {
        ImagePoints.at<float>(c,0)=points[c][0];
        ImagePoints.at<float>(c,1)=points[c][1];
    }

    cv::Mat raux,taux;
    cv::solvePnP(ObjPoints, ImagePoints, camMatrix, distCoeff,raux,taux);

    raux.convertTo(rvec,CV_32F);
    taux.convertTo(tvec,CV_32F);

    cv::Mat R,t;
    cv::Rodrigues(rvec,R);

    R = R.t();
    t = -R*tvec;

//    //TODO z_forward
//    if (_z_forward.get())
//    {
//    double temp = t.at<float>(0);
//    t.at<float>(0) = t.at<float>(2);
//    t.at<float>(2) = -t.at<float>(1);
//    t.at<float>(1) = -temp;
//    cv::Mat rvec2(3,1,CV_64FC1);
//    rvec2.at<float>(0) = rvec.at<float>(2);
//    rvec2.at<float>(2) = -rvec.at<float>(1);
//    rvec2.at<float>(1) = -rvec.at<float>(0);
//    cv::Rodrigues(rvec2,R);
//    R = R.t();
//    }

    base::Matrix3d m;
    cv::cv2eigen(R,m);
    base::Quaterniond quat(m);
    rbs.orientation = quat;
    rbs.position.x() = t.at<float>(0);
    rbs.position.y() = t.at<float>(1);
    rbs.position.z() = t.at<float>(2);

}

void Task::draw(cv::Mat &in, double p[][2], double c[], int id, cv::Scalar color, int lineWidth)const
{

    cv::line( in,cv::Point(p[0][0], p[0][1]),cv::Point(p[1][0], p[1][1]),color,lineWidth,cv::LINE_AA);
    cv::line( in,cv::Point(p[1][0], p[1][1]),cv::Point(p[2][0], p[2][1]),color,lineWidth,cv::LINE_AA);
    cv::line( in,cv::Point(p[2][0], p[2][1]),cv::Point(p[3][0], p[3][1]),color,lineWidth,cv::LINE_AA);
    cv::line( in,cv::Point(p[3][0], p[3][1]),cv::Point(p[0][0], p[0][1]),color,lineWidth,cv::LINE_AA);

    cv::rectangle( in,cv::Point(p[0][0], p[0][1]) - cv::Point(2,2),cv::Point(p[0][0], p[0][1])+cv::Point(2,2),cv::Scalar(0,0,255,255),lineWidth,cv::LINE_AA);
    cv::rectangle( in,cv::Point(p[1][0], p[1][1]) - cv::Point(2,2),cv::Point(p[1][0], p[1][1])+cv::Point(2,2),cv::Scalar(0,255,0,255),lineWidth,cv::LINE_AA);
    cv::rectangle( in,cv::Point(p[2][0], p[2][1]) - cv::Point(2,2),cv::Point(p[2][0], p[2][1])+cv::Point(2,2),cv::Scalar(255,0,0,255),lineWidth,cv::LINE_AA);

        char cad[100];
        sprintf(cad,"id=%d",id);
        //determine the centroid
        cv::Point cent(0,0);
        for (int i=0;i<4;i++)
        {
            cent.x+=c[0];
            cent.y+=c[1];
        }
        cent.x/=4.;
        cent.y/=4.;
        cv::putText(in,cad, cent,cv::FONT_HERSHEY_SIMPLEX, 1,  cv::Scalar(255-color[0],255-color[1],255-color[2],255),2);

}

void Task::draw3dCube(cv::Mat &Image,float marker_size,cv::Mat  camMatrix,cv::Mat distCoeff)
{
    cv::Mat objectPoints (8,3,CV_32FC1);
    double halfSize=marker_size/2;

      objectPoints.at<float>(0,0)=-halfSize;
      objectPoints.at<float>(0,1)=-halfSize;
      objectPoints.at<float>(0,2)=0;
      objectPoints.at<float>(1,0)=-halfSize;
      objectPoints.at<float>(1,1)=halfSize;
      objectPoints.at<float>(1,2)=0;
      objectPoints.at<float>(2,0)=halfSize;
      objectPoints.at<float>(2,1)=halfSize;
      objectPoints.at<float>(2,2)=0;
      objectPoints.at<float>(3,0)=halfSize;
      objectPoints.at<float>(3,1)=-halfSize;
      objectPoints.at<float>(3,2)=0;

      objectPoints.at<float>(4,0)=-halfSize;
      objectPoints.at<float>(4,1)=-halfSize;
      objectPoints.at<float>(4,2)=marker_size;
      objectPoints.at<float>(5,0)=-halfSize;
      objectPoints.at<float>(5,1)=halfSize;
      objectPoints.at<float>(5,2)=marker_size;
      objectPoints.at<float>(6,0)=halfSize;
      objectPoints.at<float>(6,1)=halfSize;
      objectPoints.at<float>(6,2)=marker_size;
      objectPoints.at<float>(7,0)=halfSize;
      objectPoints.at<float>(7,1)=-halfSize;
      objectPoints.at<float>(7,2)=marker_size;

    std::vector<cv::Point2f> imagePoints;
    cv::projectPoints(objectPoints, rvec,tvec,  camMatrix ,distCoeff, imagePoints);

    //draw lines of different colours
    for (int i=0;i<4;i++)
        cv::line(Image,imagePoints[i],imagePoints[(i+1)%4],cv::Scalar(0,0,255,255),1,cv::LINE_AA);

    for (int i=0;i<4;i++)
        cv::line(Image,imagePoints[i+4],imagePoints[4+(i+1)%4],cv::Scalar(0,0,255,255),1,cv::LINE_AA);

    for (int i=0;i<4;i++)
        cv::line(Image,imagePoints[i],imagePoints[i+4],cv::Scalar(0,0,255,255),1,cv::LINE_AA);

}

void Task::draw3dAxis(cv::Mat &Image, float marker_size, cv::Mat camera_matrix, cv::Mat dist_matrix)
{
	float size=marker_size*2;
    cv::Mat objectPoints (4,3,CV_32FC1);
    objectPoints.at<float>(0,0)=0;
    objectPoints.at<float>(0,1)=0;
    objectPoints.at<float>(0,2)=0;
    objectPoints.at<float>(1,0)=size;
    objectPoints.at<float>(1,1)=0;
    objectPoints.at<float>(1,2)=0;
    objectPoints.at<float>(2,0)=0;
    objectPoints.at<float>(2,1)=size;
    objectPoints.at<float>(2,2)=0;
    objectPoints.at<float>(3,0)=0;
    objectPoints.at<float>(3,1)=0;
    objectPoints.at<float>(3,2)=size;

    std::vector<cv::Point2f> imagePoints;

    cv::projectPoints( objectPoints, rvec, tvec, camera_matrix, dist_matrix, imagePoints);
    //draw lines of different colours
    cv::line(Image,imagePoints[0],imagePoints[1],cv::Scalar(0,0,255,255),1,cv::LINE_AA);
    cv::line(Image,imagePoints[0],imagePoints[2],cv::Scalar(0,255,0,255),1,cv::LINE_AA);
    cv::line(Image,imagePoints[0],imagePoints[3],cv::Scalar(255,0,0,255),1,cv::LINE_AA);
    cv::putText(Image,"x", imagePoints[1],cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,0,255,255),2);
    cv::putText(Image,"y", imagePoints[2],cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,0,255),2);
    cv::putText(Image,"z", imagePoints[3],cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255,0,0,255),2);
}

double Task::tic()
{
    struct timeval t;
    gettimeofday(&t, NULL);
    return ((double)t.tv_sec + ((double)t.tv_usec)/1000000.);
}

