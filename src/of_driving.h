#ifndef DRIVING_H
#define DRIVING_H

//System includes
#include <iostream>
#include <string.h>

//Opencv includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>


//ROS includes 
#include <ros/ros.h>
#include <std_msgs/Float64.h>

using namespace std;
using namespace cv;


class of_driving{
	
	ros::NodeHandle nh_;

	ros::Publisher theta_pub, norm_pub, steering_pub;

public:
	//Constructor
	of_driving();

	//Destructor
	~of_driving();

	//Set the size of the image used by the image processing algorithm
	void set_imgSize(int w, int h);

	//initialization of the sampling time
	inline void setTc(double T_c) {Tc = T_c;}

	inline void set_tilt(double tilt) {camera_tilt = tilt;}

	//run function - Real time image processing algorithm
	void run(Mat& img, Mat& prev_img, double, double);

	//Set the tilt and pan angles of the camera mounted on the car
	bool setPanTilt(int key, float tilt_cmd, float pan_cmd);

	//Print on the image he information about the current pan and tilt angles of the camera
	void plotPanTiltInfo(Mat& img, float tilt_cmd, float pan_cmd);

	//get functions
	inline double get_steering() {return - steering;}
	inline double get_tilt() {return tilt_angle;}
	inline double get_pan() {return pan_angle;}
	inline double get_throttle() {return ankle_angle;}

	//the control variable: steering velocity (or angular velocity?)
	double R;

	//initialize flows
	void initFlows();

private:

	//Simulation flag
	bool simulation;

	//Image size
	int img_height, img_width;

	//Pair of images
	Mat GrayImg, GrayPrevImg;

	//Sampling time
	double Tc;

	//Maximum angular velocity
	double Rm;
	//Old angular velocity value (for low-pass filtering)
	double Rold;

	double px_old, py_old;


	//linear velocity
	double linear_vel;

	double img_lowpass_freq, ctrl_lowpass_freq;

	//Car control variables
	double steering;
	double ankle_angle;
	double tilt_angle;
	double pan_angle;
	double camera_tilt;

	bool camera_set;

	//number of layer for multiresolution pyramid transform
	int maxLayer;

	//cut-off frequencies for low-pass filtering
	double cutoff_f_dp;
	double cutoff_f_vel;

	//gradient scale factor
	double grad_scale;

	//erode/dilate scale factor
	double erode_factor;
	double dilate_factor;

	//optical flow field
	Mat optical_flow, old_flow;
	vector<Mat> of_pyramid;

	//planar flow field
	Mat planar_flow;


	//dominant flow field
	Mat dominant_plane, best_plane, old_plane;
	Mat smoothed_plane;

	//Gradient vector field
	Mat gradient_field;

	//Potential Field
	Mat potential_field;

	//Control Force 
	Matx21f p_bar;

	//Navigation angle
	float theta, theta_old;

	//Affine Coefficients
	Matx22f A;
	Matx21f b;

	//Flag for Optical Flow algorithm
	bool LKpyr;

	bool control;

	//window size of the optical flow algorithm
	int windows_size;

	//Display flow resolution
	int flowResolution;

	//RANSAC inliers counter
	int point_counter, best_counter;

	//RANSAC iterations
	int iteration_num;

	//Farneback algorithm iterations
	int of_iterations;

	//RANSAC terminate condition
	int max_counter;

	//Threshold for flows comparison
	double epsilon;

	//Vehicle wheelbase
	double wheelbase;


	/** Variable used for video recording **/
	VideoWriter record;
	bool save_video;

	//flag to activate fake black corners
	bool fake_corners;

	/* Methods */
	void computeOpticalFlowField(Mat&, Mat&);

	void estimateAffineCoefficients(bool,Mat&);
	void estimatePlanarFlowField(Mat&);
	void buildDominantPlane();
	void computeGradientVectorField();
	void computePotentialField();
	void computeControlForceOrientation();
	void computeRobotVelocities(double, double);
    void displayImages(Mat&);

    void fakeBlackSpot();

	//Warping image operation
	Mat warpImage(Mat,Mat);


	double low_pass_filter(double in, double out_old, double Tc, double tau);
	double high_pass_filter(double in, double in_prev, double out_old, double Tc, double tau);

    void arrowedLine2(Mat&, cv::Point2f, cv::Point2f, const Scalar&, int thickness, int line_type, int shift, 
    double tipLength);

    void displayImagesWithName(Mat&, string name);
    void updateInitialGuessOF();



};












#endif // DRIVING_H