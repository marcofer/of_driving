#include "image_converter.h"


ImageConverter::ImageConverter(): nh_(), it_(nh_){
	key = 0;

	if(nh_.getParam("image_width", img_width))
		ROS_INFO("Image width: %d pixel", img_width);
	if(nh_.getParam("image_height", img_height))
		ROS_INFO("Image height: %d pixel", img_height);
    if(nh_.getParam("real",real))
        ROS_INFO("real %s ", real ? "true" : "false");    


	//Image subscriber
	image_vrep_sub_ = it_.subscribe("/vrep/visionSensorData",1, &ImageConverter::imageCb, this);
	raw_image_sub = it_.subscribe("/raw_images",1,&ImageConverter::rawImageCb, this);

	//Subscribe to keyboard inputs
	keyDown_sub_ = nh_.subscribe<keyboard::Key>("/keyboard/keydown",1, &ImageConverter::msgKeyDown, this);
	keyUp_sub_ = nh_.subscribe<keyboard::Key>("/keyboard/keyup",1, &ImageConverter::msgKeyUp, this);

	//Initialize time instant 
	t0 = 0.0;

	//Publishers
	tilt_pub_ = nh_.advertise<std_msgs::Float64>("/tilt",1);
	pan_pub_ = nh_.advertise<std_msgs::Float64>("/pan",1);
	alpha_pub_ = nh_.advertise<std_msgs::Float64>("/alpha",1);
	beta_pub_ = nh_.advertise<std_msgs::Float64>("/beta",1);

	/// Subscribe to joystick inputs
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 1, &ImageConverter::msgCb, this);

	//Sampling time
	double Tc = 1.0/20.0; //camera frame rate, but the value is not certain

	//Initialization
	tilt_cmd = 0.0;
	pan_cmd = 0.0;
	accelerate_cmd = 0.0;
	steer_cmd = 0.0;

	camera_set = false;


    drive.set_imgSize(img_width,img_height);
    drive.initFlows();


    image = Mat::zeros(img_height,img_width,CV_8UC3);
    prev_image = Mat::zeros(img_height,img_width,CV_8UC3);
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


	/*float t = msg->header.stamp.nsec;
	cout << "elapsed time between two image messages: " << (t - t0)*1e-9 << endl;
	cout << "image frame rate: " << 1.0/((t - t0)*1e-9) << endl;
	t0 = t;//*/

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
	run_algorithm(image,prev_image);


}


void ImageConverter::msgKeyDown(const keyboard::Key::ConstPtr& msg){

	switch(msg->code){
		case(117): // press 'u'
			tilt_cmd = 1.0;
			break;
		case(100): // press 'd'
			tilt_cmd = -1.0;
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
			break;
		case(100): //release 'd'
			tilt_cmd = 0.0;
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
			head_tilt = drive.get_tilt();
			head_pan = drive.get_pan();
			drive.set_tilt(-head_tilt);
			imshow("camera image",info_image);
		}
		else{
			cvDestroyWindow("camera image");
			drive.run(img,prev_img,accelerate_cmd,steer_cmd);
	        drive.plotPanTiltInfo(info_image,tilt_cmd,pan_cmd);
	        alpha = drive.get_steering();
	        beta = drive.get_throttle();
	        head_tilt = drive.get_tilt();
	        head_pan = drive.get_pan();
		}
	}
	else{
		drive.run(img,prev_img,accelerate_cmd,steer_cmd);		
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