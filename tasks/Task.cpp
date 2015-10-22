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

        // apply scaling to calibration parameters
        scaling = 1.0;
        if(_scale_image.value() > 0.0 && _scale_image.value() != 1.0)
            scaling = _scale_image.value();
        if(scaling != 1.0)
        {
            cal.fx = scaling * cal.fx;
            cal.fy = scaling * cal.fy;
            cal.cx = scaling * cal.cx;
            cal.cy = scaling * cal.cy;
            cal.height = scaling * cal.height;
            cal.width = scaling * cal.width;
        }

	camera_k = (cv::Mat_<double>(3,3) << cal.fx, 0, cal.cx,
                                            0, cal.fy, cal.cy,
                                            0, 0, 1);
	camera_dist = (cv::Mat_<double>(1,4) << cal.d0, cal.d1, cal.d2, cal.d3);

	// setting immutable parameters
	conf = _conf_param.get();

	rvec.create(3,1,CV_64FC1);
	tvec.create(3,1,CV_64FC1);

	// Initialize the undistortion maps:
	cv::initUndistortRectifyMap(camera_k, camera_dist, cv::Mat(), camera_k, cv::Size(cal.width, cal.height), CV_16SC2, undist_map1, undist_map2);

	// Initialize April-Detector library:
	//set the apriltag family
	tf = NULL;
	switch (conf.family)
	{
	case TAG25H7:
		tf = tag25h7_create();
		break;
	case TAG25H9:
		tf = tag25h9_create();
		break;
	case TAG36H10:
		tf = tag36h10_create();
		break;
	case TAG36H11:
		tf = tag36h11_create();
		break;
	case TAG36ARTOOLKIT:
		tf = tag36artoolkit_create();
		break;
	default:
		throw std::runtime_error("The desired apriltag family code is not implemented");
		break;
	}

	// create a apriltag detector and add the current family
	td = apriltag_detector_create();
	apriltag_detector_add_family(td, tf);
	td->quad_decimate = conf.decimate;
	td->quad_sigma = conf.blur;
	td->nthreads = conf.threads;
	td->debug = conf.debug;
	td->refine_edges = conf.refine_edges;
	td->refine_decode = conf.refine_decode;
	td->refine_pose = conf.refine_pose;

	std::vector<ApriltagIDToSize> apriltag_size_id = _apriltag_id_to_size.get();
	apriltag_id_to_size_.clear();
	
	//Initialize the id2size mapping
	for(std::vector<ApriltagIDToSize>::iterator it = apriltag_size_id.begin(); it != apriltag_size_id.end(); it++)
	{
	
	  apriltag_id_to_size_[ it->id] = it->marker_size;
	  
	}
	
	
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
		double t0 = tic();
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

                // scale image size
                if(scaling != 1.0)
                {
                    cv::Mat image_scaled;
                    cv::resize(image, image_scaled, cv::Size(), scaling, scaling);
                    image = image_scaled;
                }

		// convert to grayscale and undistort both images
		cv::Mat image_gray;
		if(image.channels() == 3)
		{
			cv::Mat temp;
			cv::remap(image, temp, undist_map1, undist_map2, cv::INTER_LINEAR);

			cv::cvtColor(temp, image_gray, cv::COLOR_RGB2GRAY);
			image = temp;
		}
		else
		{
			cv::remap(image, image_gray, undist_map1, undist_map2, cv::INTER_LINEAR);
			image = image_gray;
		}
		double t1 = tic(), t_undistort=t1-t0; t0=t1;

		// FIXME what is the use of this?
		//Repeat processing on input set this many
		int maxiters = conf.iters;
		const int hamm_hist_max = 10;

		std::vector<base::Vector2d> corners;
		//main loop
		for (int iter = 0; iter < maxiters; iter++)
		{
			if (maxiters > 1)
				printf("iter %d / %d\n", iter + 1, maxiters);
			int hamm_hist[hamm_hist_max];
			memset(hamm_hist, 0, sizeof(hamm_hist));

            //convert cv::Mat to image_u8_t
			//copying one row at once
			image_u8_t *im = image_u8_create(image_gray.cols, image_gray.rows);
			for(int i=0; i < image_gray.rows; ++i)
			{
				int jump = i*im->stride;
				memcpy(im->buf+jump, image_gray.ptr(i), image_gray.step[0]*image_gray.elemSize());
			}
			double t1 = tic(), t_prepare=t1-t0;

			//initialize time and detect markers
			zarray_t *detections = apriltag_detector_detect(td, im);
			double t2=tic();
			double t_detect = t2-t1; t1=t2;

			//build the rbs_vector and draw detected markers
			std::vector<base::samples::RigidBodyState> rbs_vector;
			for (int i = 0; i < zarray_size(detections); i++)
			{
				apriltag_detection_t *det;
				zarray_get(detections, i, &det);
				
				double size;
				
				if( apriltag_id_to_size_.find( det->id) != apriltag_id_to_size_.end())
				{
				  size = apriltag_id_to_size_[ det->id];
				}
				else
				{
				  size = _marker_size.get();
				}

				//estimate pose and push_back to rbs_vector
				base::samples::RigidBodyState rbs;
				getRbs(rbs, size, det->p, camera_k, cv::Mat());
				rbs.sourceFrame = getMarkerFrameName( det->id);
				rbs.time = current_frame_ptr->time;
				rbs_vector.push_back(rbs);

				std::cout << "APRILTAGS ID: " << det->id <<
						" x: " << rbs.position.x() <<
						" y: " << rbs.position.y() <<
						" z: " << rbs.position.z() <<
						" roll: "  << rbs.getRoll()*180/M_PI  <<
						" pitch: " << rbs.getPitch()*180/M_PI <<
						" yaw: "   << rbs.getYaw()*180/M_PI   <<
						" undistort_time: " << t_undistort*1000 << " ms " <<
						" prepare_time: " << t_prepare*1000 << " ms" <<
						" extract_time: " << t_detect*1000 << " ms" <<
						std::endl;

				for (int j=0; j < 4; ++j)
				{
					base::Vector2d aux;
					aux[0] = det->p[j][0];
					aux[1] = det->p[j][1];
					corners.push_back(aux);
				}


				if (!conf.quiet)
					printf("detection %3d: id (%2dx%2d)-%-4d, hamming %d, goodness %8.3f, margin %8.3f\n",
							i, det->family->d*det->family->d, det->family->h,
							det->id, det->hamming, det->goodness, det->decision_margin);

				if (_draw_image.get())
				{
					draw(image,det->p, det->c, det->id, cv::Scalar(0,0,255), 2);
					draw3dAxis(image, size, camera_k, cv::Mat());
					draw3dCube(image, size, camera_k, cv::Mat());
				}

				hamm_hist[det->hamming]++;

				apriltag_detection_destroy(det);
			}
			zarray_destroy(detections);

			if (!conf.quiet)
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
				_detected_corners.write(corners);
				corners.clear();
			}

			//write the markers in the output port
			if (rbs_vector.size() != 0)
			{
				_marker_poses.write(rbs_vector);
				_single_marker_pose.write( rbs_vector[0] );
				rbs_vector.clear();
			}
		}
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
	// clean-up AprilTag detector:
	apriltag_detector_destroy(td);

	switch (conf.family)
	{
	case TAG25H7:
		tag25h7_destroy(tf);
		break;
	case TAG25H9:
		tag25h9_destroy(tf);
		break;
	case TAG36H10:
		tag36h10_destroy(tf);
		break;
	case TAG36H11:
		tag36h11_destroy(tf);
		break;
	case TAG36ARTOOLKIT:
		tag36artoolkit_destroy(tf);
		break;
	default:
		throw std::runtime_error("The desired apriltag family code is not implemented");
		break;
	}

    TaskBase::cleanupHook();
}

void Task::getRbs(base::samples::RigidBodyState &rbs, float markerSizeMeters, double points[][2], cv::Mat  camMatrix,cv::Mat distCoeff)throw(cv::Exception)
{
    if (markerSizeMeters<=0)throw cv::Exception(9004,"markerSize<=0: invalid markerSize","getRbs",__FILE__,__LINE__);
    if (camMatrix.rows==0 || camMatrix.cols==0) throw cv::Exception(9004,"CameraMatrix is empty","getRbs",__FILE__,__LINE__);

    double halfSize=markerSizeMeters/2.;

    //set objects points clockwise starting from (-1,+1,0)
    // this makes the z-axis point out of the marker
    cv::Mat_<double> ObjPoints(4,3);
    ObjPoints(0,0)=-halfSize;
    ObjPoints(0,1)=+halfSize;
    ObjPoints(0,2)=0;
    ObjPoints(1,0)=+halfSize;
    ObjPoints(1,1)=+halfSize;
    ObjPoints(1,2)=0;
    ObjPoints(2,0)=+halfSize;
    ObjPoints(2,1)=-halfSize;
    ObjPoints(2,2)=0;
    ObjPoints(3,0)=-halfSize;
    ObjPoints(3,1)=-halfSize;
    ObjPoints(3,2)=0;


    //set image points from the detected points
    cv::Mat_<double> ImagePoints(4,2);
    for (int c=0;c<4;c++)
    {
        ImagePoints(c,0)=points[c][0];
        ImagePoints(c,1)=points[c][1];
    }

    // solvePnP calculates marker2camera transformation
    cv::Mat raux,taux;
    cv::solvePnP(ObjPoints, ImagePoints, camMatrix, distCoeff,raux,taux);

    raux.convertTo(rvec,CV_64F);
    taux.convertTo(tvec,CV_64F);

    cv::Mat_<double> R,t;
    cv::Rodrigues(rvec,R);

    t=tvec;

    base::Matrix3d m;
    cv::cv2eigen(R,m);
    base::Quaterniond quat(m);


    rbs.orientation = quat;
    rbs.position.x() = t(0);
    rbs.position.y() = t(1);
    rbs.position.z() = t(2);

}

void Task::draw(cv::Mat &in, double p[][2], double c[], int id, cv::Scalar color, int lineWidth)const
{

    cv::line( in,cv::Point(p[0][0], p[0][1]),cv::Point(p[1][0], p[1][1]),color,lineWidth,CV_AA);
    cv::line( in,cv::Point(p[1][0], p[1][1]),cv::Point(p[2][0], p[2][1]),color,lineWidth,CV_AA);
    cv::line( in,cv::Point(p[2][0], p[2][1]),cv::Point(p[3][0], p[3][1]),color,lineWidth,CV_AA);
    cv::line( in,cv::Point(p[3][0], p[3][1]),cv::Point(p[0][0], p[0][1]),color,lineWidth,CV_AA);

    cv::rectangle( in,cv::Point(p[0][0], p[0][1]) - cv::Point(2,2),cv::Point(p[0][0], p[0][1])+cv::Point(2,2),cv::Scalar(0,0,255,255),lineWidth,CV_AA);
    cv::rectangle( in,cv::Point(p[1][0], p[1][1]) - cv::Point(2,2),cv::Point(p[1][0], p[1][1])+cv::Point(2,2),cv::Scalar(0,255,0,255),lineWidth,CV_AA);
    cv::rectangle( in,cv::Point(p[2][0], p[2][1]) - cv::Point(2,2),cv::Point(p[2][0], p[2][1])+cv::Point(2,2),cv::Scalar(255,0,0,255),lineWidth,CV_AA);


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
        cv::line(Image,imagePoints[i],imagePoints[(i+1)%4],cv::Scalar(0,0,255,255),1,CV_AA);

    for (int i=0;i<4;i++)
        cv::line(Image,imagePoints[i+4],imagePoints[4+(i+1)%4],cv::Scalar(0,0,255,255),1,CV_AA);

    for (int i=0;i<4;i++)
        cv::line(Image,imagePoints[i],imagePoints[i+4],cv::Scalar(0,0,255,255),1,CV_AA);


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

    cv::line(Image,imagePoints[0],imagePoints[1],cv::Scalar(0,0,255,255),1,CV_AA);
    cv::line(Image,imagePoints[0],imagePoints[2],cv::Scalar(0,255,0,255),1,CV_AA);
    cv::line(Image,imagePoints[0],imagePoints[3],cv::Scalar(255,0,0,255),1,CV_AA);
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

std::string Task::getMarkerFrameName(int i)
{
    std::stringstream ss;
    ss << "apriltag_id_" << i << "_frame";
    std::string marker_frame_name = ss.str();
    
    return marker_frame_name;
}

