#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <keyboard/Key.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/image_encodings.h>
#include "of_driving.h"
#include <sensor_msgs/Joy.h>


using namespace std;
using namespace cv;


class ImageConverter{
	
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;

	//subscriber to images published by a V-REP vision sensor
	image_transport::Subscriber image_vrep_sub_, raw_image_sub;

	//keyboard subsribers
	ros::Subscriber keyDown_sub_;
	ros::Subscriber keyUp_sub_;

	/// joystick subiscriber
  	ros::Subscriber joy_sub_;

	//car-camera parameters publishers
	ros::Publisher tilt_pub_, pan_pub_, alpha_pub_, beta_pub_;

	//instance of class of_driving
	//of_driving drive;

	clock_t begin_t, end_t;
	ros::Time begin, end;
	float t0;

	of_driving drive;

public:
	//Constructor
	ImageConverter();

	//Destructor
	~ImageConverter();

private:
	void msgKeyDown(const keyboard::Key::ConstPtr& msg);
	void msgKeyUp(const keyboard::Key::ConstPtr& msg);
	void imageCb(const sensor_msgs::ImageConstPtr& msg);
	void rawImageCb(const sensor_msgs::ImageConstPtr& msg);
    void msgCb(const sensor_msgs::Joy::ConstPtr& msg);


	void run_algorithm(cv::Mat&, cv::Mat&); //image, prev_image

	//Keyboard commands
	int pan_cmd, tilt_cmd;
	double accelerate_cmd, steer_cmd;
	//value to terminate image display
	int key;

	//check if the camera has been set before starting driving
	bool camera_set;	
	bool real;
	bool NAOimg;


	//image processing parameters
	int img_width, img_height;

	//Pair of processing images
	Mat image, prev_image;

};