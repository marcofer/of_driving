#include "image_converter.h"

#include <ros/transport_hints.h>


ImageConverter::ImageConverter(): nh_(), it_(nh_){
	key = 0;

	if(nh_.getParam("image_width", img_width))
		ROS_INFO("Image width: %d pixel", img_width);
	if(nh_.getParam("image_height", img_height))
		ROS_INFO("Image height: %d pixel", img_height);
    if(nh_.getParam("real",real))
        ROS_INFO("real %s ", real ? "true" : "false");    
    if(nh_.getParam("NAOimg",NAOimg))
        ROS_INFO("NAOimg %s ", NAOimg ? "true" : "false");    
	if(nh_.getParam("fc_ratio",fc_ratio))
		ROS_INFO("fc_ratio set to %f",fc_ratio);

    string raw_src_topic;

    if(!NAOimg){
    	raw_src_topic = "/raw_images";
    }
    else{
    	raw_src_topic = "/nao_robot/camera/bottom/camera/image_raw";
    }

	//Image subscriber
	image_vrep_sub_ = it_.subscribe("/vrep/visionSensorData",1, &ImageConverter::imageCb, this, image_transport::TransportHints("raw",ros::TransportHints().tcpNoDelay()));
	raw_image_sub = it_.subscribe(raw_src_topic,1,&ImageConverter::rawImageCb, this);

	//Subscribe to keyboard inputs
	keyDown_sub_ = nh_.subscribe<keyboard::Key>("/keyboard/keydown",1, &ImageConverter::msgKeyDown, this);
	keyUp_sub_ = nh_.subscribe<keyboard::Key>("/keyboard/keyup",1, &ImageConverter::msgKeyUp, this);

	/// Subscribe to joystick inputs
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 1, &ImageConverter::msgCb, this);

	//Publishers
	tilt_pub_ = nh_.advertise<std_msgs::Float64>("/tilt",1);
	pan_pub_ = nh_.advertise<std_msgs::Float64>("/pan",1);
	alpha_pub_ = nh_.advertise<std_msgs::Float64>("/alpha",1);
	beta_pub_ = nh_.advertise<std_msgs::Float64>("/beta",1);
	hz_pub = nh_.advertise<std_msgs::Float64>("/hz",1);

	//Initialization
	rect_cmd = 0.0;
	tilt_cmd = 0.0;
	pan_cmd = 0.0;
	accelerate_cmd = 0.0;
	steer_cmd = 0.0;
	camera_set = false;

    drive.set_imgSize(img_width,img_height);
    drive.initFlows();

    image = Mat::zeros(img_height,img_width,CV_8UC3);
    prev_image = Mat::zeros(img_height,img_width,CV_8UC3);

    gettimeofday(&start_tod,NULL);
    elapsed_tod = 0.0;
    cout << "elapsed_tod: " << elapsed_tod << endl;	
}

ImageConverter::~ImageConverter(){
    cv::destroyAllWindows();
}


void ImageConverter::msgCb(const sensor_msgs::Joy::ConstPtr& joy)
{

    /// Get joystick commands to be used (eventually) for teleoperation
    accelerate_cmd = joy->axes[1];
    steer_cmd = -joy->axes[3];

}

void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg){
	
	//Start counting the time interval required to run an algorithm cycle
	//begin_t = clock();

	cv_bridge::CvImagePtr cv_ptr;

	try{
		cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
	}
	catch(cv_bridge::Exception& e){
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}

	//Update the Cv matrixes 
	image.copyTo(prev_image);
	cv_ptr->image.copyTo(image);

	//run the main algorithm
	run_algorithm(image,prev_image);

    gettimeofday(&end_tod,NULL);
    elapsed_tod = (end_tod.tv_sec + (double)end_tod.tv_usec /1000000.0)
		  - (start_tod.tv_sec + (double)start_tod.tv_usec/1000000.0);
	start_tod = end_tod;

	//cout << "time: " << elapsed_tod << "s" << endl;
	//cout << "frequency: " << 1.0/elapsed_tod << "Hz" << endl << endl;

	double cutoff_f = 1.0/elapsed_tod*0.25;


}

void ImageConverter::rawImageCb(const sensor_msgs::ImageConstPtr& msg){
	cv_bridge::CvImagePtr cv_ptr;

	try{
		cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
	}
	catch(cv_bridge::Exception& e){
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}

	//Update the Cv matrixes 
	image.copyTo(prev_image);
	cv_ptr->image.copyTo(image);

	//cvtColor(image,image,CV_BGR2GRAY);
	//imshow("Test",cv_ptr->image);
	//cvWaitKey(1);
	//run the main algorithm

    gettimeofday(&end_tod,NULL);

    elapsed_tod = (end_tod.tv_sec + (double)end_tod.tv_usec /1000000.0)
		  - (start_tod.tv_sec + (double)start_tod.tv_usec/1000000.0);

	start_tod = end_tod;
	double cutoff_f = 1.0/elapsed_tod*fc_ratio;

	std_msgs::Float64 hz_msg;
	hz_msg.data = 1.0/elapsed_tod;
	hz_pub.publish(hz_msg);
	/*cout << "[image_converter] elapsed_tod: " << elapsed_tod << endl;
	cout << "[image_converter] freq_tod: " << 1.0/elapsed_tod << endl << endl;//*/

	//cout << "Tc: " << sampling_time << "---> frequency: " << 1.0/sampling_time << endl;
	//cout << "cut off f: " << cutoff_f << endl << endl;
	drive.setTc(elapsed_tod);
	drive.setImgLowPassFrequency(cutoff_f);
	drive.setBarLowPassFrequency(cutoff_f*fc_ratio*0.5);

	run_algorithm(image,prev_image);
}


void ImageConverter::msgKeyDown(const keyboard::Key::ConstPtr& msg){

	switch(msg->code){
		case(117): // press 'u'
			tilt_cmd = 1.0;
			rect_cmd = 1.0;
			break;
		case(100): // press 'd'
			tilt_cmd = -1.0;
			rect_cmd = -1.0;
			break;
		case(114): // press 'r'
			pan_cmd = -1.0;
			break;
		case(108): // press 'l'
			pan_cmd = 1.0;
			break;
		default:
			pan_cmd = 0.0;
			tilt_cmd = 0.0;
			break;
	}
}

void ImageConverter::msgKeyUp(const keyboard::Key::ConstPtr& msg){

	switch(msg->code){
		case(117): //release 'u'
			tilt_cmd = 0.0;
			rect_cmd = 0.0;
			break;
		case(100): //release 'd'
			tilt_cmd = 0.0;
			rect_cmd = 0.0;
			break;
		case(114): //release 'r'
			pan_cmd = 0.0;
			break;
		case(108): //release 'l'
			pan_cmd = 0.0;
			break;
	}
}

void ImageConverter::run_algorithm(Mat& img, Mat& prev_img){
	
	Mat info_image;
	img.copyTo(info_image);

	double head_tilt = -0.3;
	double head_pan = 0.0;

	float beta = -1;
	float alpha = 0.0;

	if(!real){
		if(!camera_set && !image.empty()){
			camera_set = drive.setPanTilt(key,tilt_cmd,pan_cmd);
			drive.plotPanTiltInfo(info_image,tilt_cmd,pan_cmd);
			drive.setTc(0.05);
			drive.setImgLowPassFrequency(10.0);
			drive.setBarLowPassFrequency(10.0);
			head_tilt = drive.get_tilt();
			head_pan = drive.get_pan();
			drive.set_tilt(-head_tilt);
			imshow("camera image",info_image);
		}
		else{
			cvDestroyWindow("camera image");
			drive.run(img,prev_img,accelerate_cmd,steer_cmd,real);
	        drive.plotPanTiltInfo(info_image,tilt_cmd,pan_cmd);
	        alpha = drive.get_steering();
	        beta = drive.get_throttle();
	        head_tilt = drive.get_tilt();
	        head_pan = drive.get_pan();
		}
	}
	else{
		drive.setRectHeight(rect_cmd);
		drive.run(img,prev_img,accelerate_cmd,steer_cmd,real);	
	}

	key = cvWaitKey(1)%256;

	std_msgs::Float64 msg;
	msg.data = head_tilt;
	tilt_pub_.publish(msg);

	msg.data = head_pan;
	pan_pub_.publish(msg);

	msg.data = alpha;
	alpha_pub_.publish(msg);

	msg.data = beta;
	beta_pub_.publish(msg);


}