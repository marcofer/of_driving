#include "of_driving.h"
#include "math.h"


of_driving::of_driving(): nh_(){

	if(nh_.getParam("Rm",Rm))
		ROS_INFO("Rm set to %f",Rm);
	if(nh_.getParam("linear_vel", linear_vel))
		ROS_INFO("linear velocity set to %f m/s", linear_vel);
    if(nh_.getParam("img_lowpass_freq",img_lowpass_freq))
        ROS_INFO("img_lowpass_freq set to %f ", img_lowpass_freq);
    if(nh_.getParam("ctrl_lowpass_freq",ctrl_lowpass_freq))
        ROS_INFO("ctrl_lowpass_freq set to %f ", ctrl_lowpass_freq);
    if(nh_.getParam("grad_scale",grad_scale))
        ROS_INFO("grad_scale set to %f ", grad_scale);    
    if(nh_.getParam("LKpyr",LKpyr))
        ROS_INFO("LKpyr %s ", LKpyr ? "true" : "false");    
    if(nh_.getParam("control",control))
        ROS_INFO("control %s ", control ? "true" : "false");    
    if(nh_.getParam("windows_size",windows_size))
        ROS_INFO("windows_size set to %d ", windows_size);    
    if(nh_.getParam("maxLayer",maxLayer))
        ROS_INFO("maxLayer set to %d ", maxLayer);  
    if(nh_.getParam("epsilon",epsilon))
        ROS_INFO("epsilon set to %f ", epsilon);    
    if(nh_.getParam("flowResolution",flowResolution))
        ROS_INFO("flowResolution set to %d ", flowResolution);  
    if(nh_.getParam("iteration_num",iteration_num))
        ROS_INFO("iteration_num set to %d ", iteration_num);  
    if(nh_.getParam("of_iterations",of_iterations))
        ROS_INFO("of_iterations set to %d ", of_iterations);  
    if(nh_.getParam("wheelbase",wheelbase))
        ROS_INFO("wheelbase set to %f ", wheelbase);    
    if(nh_.getParam("simulation",simulation))
        ROS_INFO("simulation %s ", simulation ? "true" : "false");    
    if(nh_.getParam("save_video",save_video))
        ROS_INFO("save_video %s ", save_video ? "true" : "false");    
    if(nh_.getParam("fake_corners",fake_corners))
        ROS_INFO("fake_corners %s ", fake_corners ? "true" : "false");    
    if(nh_.getParam("erode_factor",erode_factor))
        ROS_INFO("erode_factor set to %f ", erode_factor);    
    if(nh_.getParam("dilate_factor",dilate_factor))
        ROS_INFO("dilate_factor set to %f ", dilate_factor); 

    setTc(0.05); //20Hz

	Rold = 0.0;
	px_old = 0.0;
	py_old = 0.0;

	steering = 0.0;
	ankle_angle = 0.0;
	pan_angle = 0.0;
	tilt_angle = 0.0;

	camera_set = false;

	A = Matx22f(1,0,0,1);
	b = Matx21f(0,0);

	p_bar = Matx21f(0,0);

	theta = 0.0;
	theta_old = 0.0;

	theta_pub = nh_.advertise<std_msgs::Float64>("/theta",1);
	norm_pub = nh_.advertise<std_msgs::Float64>("/norm",1);
	steering_pub = nh_.advertise<std_msgs::Float64>("/steering",1);
}

of_driving::~of_driving(){
	destroyAllWindows();

}

void of_driving::set_imgSize(int w, int h){
	img_width = w;
	img_height = h;
}

void of_driving::initFlows(){
	optical_flow = Mat::zeros(img_height,img_width,CV_32FC2);
	old_flow = Mat::zeros(img_height,img_width,CV_32FC2);
	dominant_plane = Mat::zeros(img_height,img_width,CV_8UC1);
	old_plane = Mat::zeros(img_height,img_width,CV_8UC1);
	smoothed_plane = Mat::zeros(img_height,img_width,CV_8UC1);
	best_plane = Mat::zeros(img_height,img_width,CV_8UC1);
	planar_flow = Mat::zeros(img_height,img_width,CV_32FC2);
	gradient_field = Mat::zeros(img_height,img_width,CV_32FC2);
	potential_field = Mat::zeros(img_height,img_width,CV_32FC2);
	point_counter = 0;
	best_counter = 0;
	max_counter = img_height*img_width/2;


	if(save_video){
		record.open("NAOcamera.avi", CV_FOURCC('D','I','V','X'),20.0, cvSize(img_width,img_height), true);
		if( !record.isOpened() ) {
			cout << "[NAO] VideoWriter failed to open!" << endl;
			}
	}

}


bool of_driving::setPanTilt(int key, float tilt_cmd, float pan_cmd){
	float delta = 0.01;

	tilt_angle -= delta*tilt_cmd;
	if(abs(tilt_angle) < delta) 
		tilt_angle = 0.0;
	else if(tilt_angle < -0.52){
		tilt_angle = -0.52;
	}
	else if(tilt_angle > 0.7 ){
		tilt_angle = 0.7;
	}
	camera_tilt = tilt_angle;

	pan_angle += delta*pan_cmd;
	if(abs(pan_angle) < delta){
		pan_angle = 0.0;
	}
	else if(pan_angle < -0-75){
		pan_angle = - 0.75;
	}
	else if(pan_angle > 0.75){
		pan_angle = 0.75;
	}

	if(key==32){
		camera_set = true;
	}

	return camera_set;
}

void of_driving::plotPanTiltInfo(Mat& img, float tilt_cmd, float pan_cmd){

	string text_str;
	ostringstream convert;
	Size text_size;

	text_str = "";
	text_str = "TILT (rad)";
	text_size = getTextSize(text_str,1,1,1,0);
	putText(img, text_str,Point(img_width-150,img_height - 40 + 0.5*text_size.height),1,1,Scalar(255,255,255),1,CV_AA);
   

    text_str = "";
    convert << setprecision(4) << tilt_angle;
    text_str = convert.str();
    putText(img,text_str,Point(img_width-150+text_size.width+10,img_height-40+0.5*text_size.height),1,1,Scalar(255,255,255),1,CV_AA);

    text_str = "";
    text_str = "PAN (rad)";
    text_size = getTextSize(text_str,1,1,1,0);
    putText(img,text_str,Point(img_width-150,img_height-20+0.5*text_size.height),1,1,Scalar(255,255,255),1,CV_AA);

    text_str = "";
    convert.str(""); convert.clear();
    convert << setprecision(4) << pan_angle;
    text_str = convert.str();
    putText(img,text_str,Point(img_width-150+text_size.width+10,img_height-20+0.5*text_size.height),1,1,Scalar(255,255,255),1,CV_AA);


    if(tilt_cmd==1){
        circle(img,Point(img_width-180,img_height-40),2,Scalar(0,0,255),2,CV_AA);
    }else if(tilt_cmd==-1){
        circle(img,Point(img_width-180,img_height-20),2,Scalar(0,0,255),2,CV_AA);
    }

    if(pan_cmd==1){
        circle(img,Point(img_width-190,img_height-30),2,Scalar(0,0,255),2,CV_AA);
    }else if(pan_cmd==-1){
        circle(img,Point(img_width-170,img_height-30),2,Scalar(0,0,255),2,CV_AA);
    }


}

void of_driving::run(Mat& img, Mat& prev_img, double acc, double steer){


	vector<Mat> pyr, prev_pyr, u_pyr;
	int k = 0;
	best_counter = 0;

	if(save_video){
		record.write(img);
	}

	cvtColor(img,GrayImg,CV_BGR2GRAY);
	cvtColor(prev_img,GrayPrevImg,CV_BGR2GRAY);


	/// ---  1. Compute the optical flow field u(x,y,t) at layer l (output: optical_flow matrix)
	computeOpticalFlowField(GrayPrevImg,GrayImg);

	while(point_counter <= max_counter && k < iteration_num){

        // --- 2. Compute affine coefficients by random selection of three points 
		estimateAffineCoefficients(false,GrayPrevImg);
		
        // --- 3. Estimate planar flow from affine coefficients
		estimatePlanarFlowField(GrayPrevImg);	
		
		// --- 4. Match the computed optical flow and esitmated planar flow, so detect the dominant plane. If the dominant plane occupies
        // less than half of the image, then go to step (2)
		buildDominantPlane();
		
        
		if(point_counter >= best_counter){
			best_counter = point_counter;
			dominant_plane.copyTo(best_plane);
		}

		k++;
	}

	if(point_counter <= max_counter){
		best_plane.copyTo(dominant_plane);
	}

    /// --- 2. Robust computation of affine coefficients with all the points belonging to the detected dominant plane 	
    estimateAffineCoefficients(true,GrayPrevImg);

	/// --- 3. Estimate planar flow from affine coefficients
	estimatePlanarFlowField(GrayPrevImg);

    /// --- 4. Match the computed optical flow and esitmated planar flow, so detect the dominant plane
    buildDominantPlane();

    if(fake_corners){
    	fakeBlackSpot();
    }

    /// --- 5. Compute gradient vector field from dominant plane 
    computeGradientVectorField();

    /// --- 6. Compute the desired planar flow, from the theta_d
    computePotentialField();

    /// --- 7. Compute the control force as average of potential field
    computeControlForceOrientation(); //to be implemented

    /// --- 8. Compute the translational and rotational robot velocities
    computeRobotVelocities(acc, steer); //to be implemented
	//*/
    /// --- END. Show the intermediate steps
	displayImages(prev_img);//*/
    cvWaitKey(1);

}

void of_driving::computeOpticalFlowField(Mat& prevImg, Mat& img){

	vector<Mat> pyr, prev_pyr;
	vector<Point2f> prevPts, nextPts;//, nextPts;
	vector<int> row_vec, col_vec;
	Size winSize(windows_size,windows_size); 
	string pyrLev;
	ostringstream convert;
	Mat status, err;
	double pyr_scale = 0.5;
	int winsize = windows_size;
	
	/* Using Farneback, the value 'iterations' is very important: it reveals to be a tradeoff between the "homogeneity" of the field
	   and the magnitude. The higher the iterations, the higher the magnitude and the lower the homogeneity of the field. */
	int poly_n = 5;
	double poly_sigma = 1.1;
	int flags = OPTFLOW_FARNEBACK_GAUSSIAN;
	string layer = "Layer ";
	bool withDerivatives = false;
	int pyr_idx, incr;

	(withDerivatives) ? (pyr_idx = maxLayer * 2) : (pyr_idx = maxLayer) ;
	(withDerivatives) ? (incr = 2) : (incr =  1) ;	
 
	buildOpticalFlowPyramid(prevImg, prev_pyr,winSize,maxLayer,withDerivatives,BORDER_REPLICATE,BORDER_REPLICATE,true);
	buildOpticalFlowPyramid(img, pyr,winSize,maxLayer,withDerivatives,BORDER_REPLICATE,BORDER_REPLICATE,true);

	if(LKpyr){ // SPARSE - LUCAS-KANADE

		//pick samples from the first image to compute sparse optical flow
	    for (int i = 0 ; i < img.rows ; i += flowResolution){
	        for (int j = 0 ; j < img.cols ; j += flowResolution){
	            prevPts.push_back(Point2f(j,i));	            
	        }
	    }
		//calcOpticalFlowPyrLK(prevImg, img, prevPts, nextPts, status, err, winSize, 1);
    	calcOpticalFlowPyrLK(prev_pyr, pyr, prevPts, nextPts, status, err, winSize, maxLayer);

		//fill the optical flow matrix with velocity vectors ( = difference of points in the two images)
	    
	    int sampled_i = img_height/flowResolution;
	    int sampled_j = img_width/flowResolution;

	    /*cout << "img_height: " << img_height << endl;
	    cout << "img_height: " << img_width << endl;
	    cout << "img_height: " << sampled_i << endl;
	    cout << "img_height: " << sampled_j << endl;//*/

	    for (int i = 0 ; i < sampled_i ; i ++){
	    	for (int j = 0 ; j < sampled_j ; j ++){
	    		int idx = i * sampled_j + j ;
	    		
	    			Point2f p(nextPts[idx] - prevPts[idx]);
	    			Mat temp(flowResolution,flowResolution,CV_32FC2,Scalar(p.x,p.y));
	    			if((j*flowResolution + flowResolution <= img_width) && (i*flowResolution + flowResolution) <= img_height)
	    					temp.copyTo(optical_flow(Rect(j*flowResolution,i*flowResolution,flowResolution,flowResolution)));
	    		 
	    	}
	    }

	    /*for (int i = 0 ; i < img.rows ; i ++){
	        for (int j = 0 ; j < img.cols ; j ++){
	            //int idx = (int)((i*img.cols)/(flowResolution*flowResolution) + j/flowResolution);
	            //if (idx < img_height*img_width/(flowResolution*flowResolution)){
		        int idx = i*img.cols + j;
		        if (idx < nextPts.size()){
		        	if(isnan(nextPts[idx].x) || isnan(nextPts[idx].y)){
		        		cout << "A VALUE IS A NAN!!!!!!" << endl ; 
		        	}
		        	else{
		            	Point2f p(nextPts[idx] - prevPts[idx]);
		        		optical_flow.at<Point2f>(i,j) = p;
		        	}
		        }
	        }
	    }//*/


   }
    else{ // DENSE - FARNEBACK
    	calcOpticalFlowFarneback(prevImg, img, optical_flow, pyr_scale, maxLayer, winsize, of_iterations, poly_n, poly_sigma, flags);		

    	/*for (int i = 0 ; i < optical_flow.rows ; i ++){
    		for (int j = 0 ; j < optical_flow.cols ; j ++){
    			Point2f p(optical_flow.at<Point2f>(i,j));
    			if (isnan(p.x) || isnan(p.y)){
    				cout << "There is a NAN!!!!!!!!!" << endl;
    				optical_flow.at<Point2f>(i,j) = Point2f(0,0);
    			}
    		}
    	}//*/
   }

   /*** LOW PASS FILTERING ***/
   	double cutf = 15.0;
	for (int i = 0 ; i < optical_flow.rows ; i ++){
		for (int j = 0 ; j < optical_flow.cols ; j ++){
			Point2f p(optical_flow.at<Point2f>(i,j));
			Point2f oldp(old_flow.at<Point2f>(i,j));
			optical_flow.at<Point2f>(i,j).x = low_pass_filter(p.x,oldp.x,Tc,1.0/img_lowpass_freq);
			optical_flow.at<Point2f>(i,j).y = low_pass_filter(p.y,oldp.y,Tc,1.0/img_lowpass_freq);
			//optical_flow.at<Point2f>(i,j).x = low_pass_filter(p.x,oldp.x,Tc,1.0/cutf);
			//optical_flow.at<Point2f>(i,j).y = low_pass_filter(p.y,oldp.y,Tc,1.0/cutf);

		}
	}

	optical_flow.copyTo(old_flow);//*/

}


void of_driving::estimateAffineCoefficients(bool robust, Mat& gImg){


	vector<Point2f> prevSamples, nextSamples;
	int i, j;
	Mat Ab;

	if(!robust){
		for (int k = 0 ; k < 3 ; k ++){
			i = rand()%img_height;
			j = rand()%img_width;
			Point2f p(j,i);
			Point2f p2(optical_flow.at<Point2f>(p) + p);
			prevSamples.push_back(p);
			nextSamples.push_back(p2);
		}

		Ab = getAffineTransform(prevSamples,nextSamples);
	}
	else{
		unsigned char *input = (unsigned char*)(dominant_plane.data);
		for (int i = 0 ; i < img_height ; i ++){
			for (int j = 0 ; j < img_width ; j ++){
				//Scalar dp = dominant_plane.at<uchar>(i,j);
				int val = input[img_width*i + j];
				//cout << "val: " << val << endl;
				if(val == 255/*dp.val[0]==100//*/){
					Point2f p(j,i);
					Point2f p2(optical_flow.at<Point2f>(p) + p);
					prevSamples.push_back(p);
					nextSamples.push_back(p2);
				}
			}
		}

		if(!prevSamples.empty() && ! nextSamples.empty())
			Ab = estimateRigidTransform(prevSamples,nextSamples,true);

	}

	if(!Ab.empty()){
		A = Ab(Rect(0,0,2,2)).clone();
		b = Ab(Rect(2,0,1,2)).clone();
	}
}

void of_driving::estimatePlanarFlowField(Mat& gImg){


	for (int i = 0 ; i < img_height ; i ++){
		for (int j = 0 ; j < img_width ; j ++){
			Matx21f p(j,i);
			Matx21f planar_vec(A*p + b - p);
			Point2f planar_p(planar_vec(0),planar_vec(1));
			planar_flow.at<Point2f>(i,j) = planar_p;
		}
	}


}


void of_driving::buildDominantPlane(){

	double edsize = 10;

	point_counter = 0;

	for (int i = 0 ; i < img_height ; i ++){
		for (int j = 0 ; j < img_width ; j ++){
			Point2f p(j,i);
			Point2f xdot(optical_flow.at<Point2f>(p));
			Point2f xhat(planar_flow.at<Point2f>(p));
			
			if(simulation && norm(xdot) < 1e-3){
				dominant_plane.at<uchar>(i,j) = 0.0;
			}
			else{
				if(norm(xdot - xhat) < epsilon){
					dominant_plane.at<uchar>(i,j) = 255;
				}
				else{
					dominant_plane.at<uchar>(i,j) = 0.0;
				}
			}
		}
	}


	erode(dominant_plane, dominant_plane, getStructuringElement(MORPH_ELLIPSE, Size(erode_factor,erode_factor)));//*/
	dilate(dominant_plane, dominant_plane, getStructuringElement(MORPH_ELLIPSE, Size(dilate_factor,dilate_factor)));//*/

	//test for books images
	//dilate(dominant_plane, dominant_plane, getStructuringElement(MORPH_ELLIPSE, Size(dilate_factor,dilate_factor)));//*/
	//erode(dominant_plane, dominant_plane, getStructuringElement(MORPH_ELLIPSE, Size(erode_factor,erode_factor)));//*/
	
	unsigned char* dpPtr = (unsigned char*)(dominant_plane.data);
	unsigned char* oldPtr = (unsigned char*)(old_plane.data);
	for (int i = 0 ; i < img_height ; i ++){
		for (int j = 0 ; j < img_width ; j ++){
				int in = dpPtr[img_width*i + j];
				int out_old = oldPtr[img_width*i + j];
				dominant_plane.at<uchar>(i,j) = low_pass_filter(in,out_old,Tc,1.0/img_lowpass_freq);
		}
	}	
	dominant_plane.copyTo(old_plane);//*/

	double thresh = 100; //100
	double maxVal = 255;
	threshold(dominant_plane,dominant_plane,thresh,maxVal,THRESH_BINARY);//*/


    for (int i = 0 ; i < img_height ; i ++){
    	for (int j = 0 ; j < img_width ; j ++){
    		Scalar dp = dominant_plane.at<uchar>(i,j);
    		if(dp.val[0] == 255){
    			point_counter ++;
    		}
    	}
    }


}


void of_driving::computeGradientVectorField(){

	Size GaussSize(51,51);
	Mat grad_x, grad_y;
	double sigmaX = img_width*img_height*0.5;
	int ddepth = CV_32F; //CV_16S
	double scale = grad_scale;
	double delta = 0.0;

	
	GaussianBlur(dominant_plane,smoothed_plane,GaussSize,sigmaX,0);
 
    Scharr(smoothed_plane, grad_x, ddepth, 1, 0, scale);
    Scharr(smoothed_plane, grad_y, ddepth, 0, 1, scale);

    for (int i = 0 ; i < img_height ; i ++){
    	for (int j = 0 ; j < img_width ; j ++){
    		Point2f p(j,i);
    		Scalar xs = grad_x.at<float>(i,j);
    		Scalar ys = grad_y.at<float>(i,j);
    		float x = xs.val[0];
    		float y = ys.val[0];
    		gradient_field.at<Point2f>(p) = Point2f(x,y);
    	}
    }

}


void of_driving::computePotentialField(){

	for (int i = 0 ; i < img_height ; i ++){
		for (int j = 0 ; j < img_width ; j ++){
			Scalar dp = dominant_plane.at<uchar>(i,j);
			if(dp.val[0] == 255){
				potential_field.at<Point2f>(i,j) = gradient_field.at<Point2f>(i,j) - planar_flow.at<Point2f>(i,j);
			}
			else{
				potential_field.at<Point2f>(i,j) = gradient_field.at<Point2f>(i,j);
			}
		}
	}//*/

}

void of_driving::computeControlForceOrientation(){
	Mat ROI;

	//ROI = potential_field(Rect(0,0/*img_height/4//*/,img_width,img_height/2)).clone();
	ROI = gradient_field(Rect(0,0/*img_height/4//*/,img_width,img_height/2)).clone();
	//ROI = gradient_field.clone();

	Mat img = Mat::ones(ROI.rows,ROI.cols,CV_8UC1)*255;
	//cvtColor(img,img,CV_GRAY2BGR);
	p_bar = Matx21f(0,0);

	for (int i = 0 ; i < ROI.rows ; i ++){
		for (int j = 0 ; j < ROI.cols ; j ++){
			Matx21f vec(ROI.at<Point2f>(i,j).x,ROI.at<Point2f>(i,j).y);
			p_bar += vec;
		}
	}

	for (int i = 0 ; i < ROI.rows ; i += flowResolution){
		for (int j = 0 ; j < ROI.cols ; j += flowResolution){
			Point2f p(j,i);
			Point2f ppo(p+ROI.at<Point2f>(p));
			arrowedLine2(img,p,ppo,Scalar(0,0,0),1.5,8,0,0.1);
		}
	}

	//imshow("Potential ROI", img);

	p_bar *= (1.0/(ROI.rows*ROI.cols));
	Point2f pbar(p_bar(0),p_bar(1));

	pbar.x  = low_pass_filter(pbar.x,px_old,Tc,1.0/ctrl_lowpass_freq);
    pbar.y  = low_pass_filter(pbar.y,py_old,Tc,1.0/ctrl_lowpass_freq);

	px_old = pbar.x;
	py_old = pbar.y;


	Point2f y(0,-1);

	if(norm(pbar) > 0.25){
		theta = pbar.dot(y);
		theta /= (norm(pbar)*norm(y));
		theta = acos(theta);
        if (p_bar(0) < 0){
    		theta *= -1;
		}

	}
	else{
		theta = 0.0;
	}

	//theta = low_pass_filter(theta,theta_old,Tc,1.0/ctrl_lowpass_freq);
	theta_old = theta;

	//Publish the navigation angle on a ROS topic (for visualization)
	std_msgs::Float64 msg;
	msg.data = theta*180.0/CV_PI;
	theta_pub.publish(msg);//*/

	std_msgs::Float64 norm_msg;
	norm_msg.data = norm(pbar);
	norm_pub.publish(norm_msg);
}


void of_driving::computeRobotVelocities(double acc, double steer){

	double R = Rm*sin(theta);

	//R = low_pass_filter(R,Rold,Tc,1.0/ctrl_lowpass_freq);
	//Rold = R;

	if(ankle_angle != -1){
        steering = wheelbase * R/linear_vel;
    }
    else{
        steering = 0.0;
    }

	//ankle_angle = acc - 1.0;
	ankle_angle = 0.0;

	std_msgs::Float64 steer_msg;
	steer_msg.data = steering;
	steering_pub.publish(steer_msg);
}


void of_driving::fakeBlackSpot(){

	Mat sp;
	smoothed_plane.copyTo(sp);
	Rect roi_left(0,0,img_width/8,img_height/8);
	Rect roi_right(7.0/8.0*img_width,0,img_width/8,img_height/8);

	Mat left_corner(dominant_plane,roi_left);
	Mat right_corner(dominant_plane,roi_right);

	Scalar avgL, avgR;

	int ths = 50;
	int value = 0;

	avgL = mean(left_corner);
	avgR = mean(right_corner);

	if (avgL.val[0] > ths){
		for (int i = 0 ; i < left_corner.rows ; i ++){
			for (int j = 0 ; j < left_corner.cols ; j ++){
				left_corner.at<uchar>(i,j) = value;
			}
		}
	}

	if (avgR.val[0] > ths){
		for (int i = 0 ; i < right_corner.rows ; i ++){
			for (int j = 0 ; j < right_corner.cols ; j ++){
				right_corner.at<uchar>(i,j) = value;
			}
		}
	}


}

void of_driving::displayImages(Mat& img){

	string ofAlg;
	Mat gImg, u_img, p_img, gradient_img, potential_img, cf_img;
	Point2f center(img.cols/2,img.rows/2);
	Point2f pb(p_bar(0),p_bar(1));
	Point2f px(pb.x*20.0,0);
	Point2f y(0,-1);

	double pbnorm = norm(px);
	double pbnorm_max = 50;

	if (pbnorm > pbnorm_max){
		pbnorm = pbnorm_max;
		px.x = boost::math::sign(px.x)*50;
	}

	double red = pbnorm/pbnorm_max*255.0 ;
	double green = 255.0 - red;

	LKpyr ? ofAlg = "Lucas-Kanade" : ofAlg = "Farneback";
	img.copyTo(cf_img);
	potential_img = Mat::ones(img_height,img_width,CV_8UC1)*255;

	try{
	cvtColor(img,gImg,CV_BGR2GRAY);
	cvtColor(gImg,u_img,CV_GRAY2BGR);
	cvtColor(gImg,p_img,CV_GRAY2BGR);
    }
    catch(cv::Exception &e){
    	cerr << "cvtColor in displayImages" << endl;
    }
    
    smoothed_plane.copyTo(gradient_img);
    cvtColor(gradient_img,gradient_img,CV_GRAY2BGR);


	/*** Original color image ***/
	//imshow("color image", img);

    /*** Optical Flow - Planar Flow - Gradient Field - Potential Field***/
    for (int i = 0 ; i < img.rows ; i += flowResolution*2){
 	   for (int j = 0 ; j < img.cols ; j += flowResolution*2){
    	    Point2f p(j,i);
        	Point2f po(p+optical_flow.at<Point2f>(Point2f(j,i)));
			Point2f pp(p+planar_flow.at<Point2f>(Point2f(j,i)));
			Point2f pg(p+gradient_field.at<Point2f>(p)*0.1);
			Point2f norm_pg((p + pg)*(1.0/norm(p + pg)));
			Point2f ppo(p+potential_field.at<Point2f>(p));
        	arrowedLine2(u_img,p,po,Scalar(0,0,255),1.5,8,0,0.1);
			arrowedLine2(p_img,p,pp,Scalar(255,255,0),1.5,8,0,0.1);
			arrowedLine2(gradient_img,p,pg,Scalar(0,255,0),1.5,8,0,0.1);
			arrowedLine2(potential_img,p,ppo,Scalar(0,0,0),1.5,8,0,0.1);
	    } 
	}

	/*** Control Force***/

	arrowedLine2(cf_img,center,center + y*50,Scalar(255,0,0),3.0,8,0,0.1);
	arrowedLine2(cf_img,center,center + px,Scalar(0,green,red),3.0,8,0,0.1);

	Mat total = Mat::zeros(3*img_height,2*img_width,CV_8UC3);
	Mat dp_img, sp_img;
	cvtColor(dominant_plane,dp_img,CV_GRAY2BGR);
	cvtColor(smoothed_plane,sp_img,CV_GRAY2BGR);
	cvtColor(potential_img,potential_img,CV_GRAY2BGR);

	u_img.copyTo(total(Rect(0,0,img_width,img_height)));
	p_img.copyTo(total(Rect(img_width,0,img_width,img_height)));
	dp_img.copyTo(total(Rect(0,img_height,img_width,img_height)));
	sp_img.copyTo(total(Rect(img_width,img_height,img_width,img_height)));
	gradient_img.copyTo(total(Rect(0,2*img_height,img_width,img_height)));
	cf_img.copyTo(total(Rect(img_width,2*img_height,img_width,img_height)));

    //imshow(ofAlg,u_img);
    //imshow("Planar flow", p_img);
	//imshow("Dominant Plane", dominant_plane);
	//imshow("Filtered dominant plane",smoothed_plane);
	//imshow("Gradient Vector Field",gradient_img);
	//imshow("Potential Field",potential_img);
	//imshow("Control Force",cf_img);

	imshow("Steps",total);

}



/** Auxiliary functions **/

double of_driving::low_pass_filter(double in, double out_old, double Tc, double tau){

    //cout << "filtering" << endl;

    double out, alpha;

    alpha = 1 / ( 1 + (tau/Tc));

    out = alpha * in + (1-alpha) * out_old;

    return out;

}

double of_driving::high_pass_filter(double in, double in_prev, double out_old, double Tc, double tau){

    //cout << "filtering" << endl;

    double out, alpha;

    alpha = 1 / ( 1 + (tau/Tc));

    out = alpha *  out_old + alpha * (in - in_prev);

    return out;

}


void of_driving::arrowedLine2(Mat& img, cv::Point2f pt1, cv::Point2f pt2, const Scalar& color, int thickness, int line_type, int shift, 
    double tipLength)
{
    const double tipSize = norm(pt1-pt2)*tipLength; // Factor to normalize the size of the tip depending on the length of the arrow
    line(img, pt1, pt2, color, thickness, line_type, shift);
    const double angle = atan2( (double) pt1.y - pt2.y, (double) pt1.x - pt2.x );
    Point p(cvRound(pt2.x + tipSize * cos(angle + CV_PI / 4)),
    cvRound(pt2.y + tipSize * sin(angle + CV_PI / 4)));
    line(img, p, pt2, color, thickness, line_type, shift);
    p.x = cvRound(pt2.x + tipSize * cos(angle - CV_PI / 4));
    p.y = cvRound(pt2.y + tipSize * sin(angle - CV_PI / 4));
    line(img, p, pt2, color, thickness, line_type, shift);
}//*/

void of_driving::updateInitialGuessOF(){
	for (int j = 0 ; j < (optical_flow).rows ; j += flowResolution){
		for (int k = 0 ; k < (optical_flow).cols ; k += flowResolution){
			Point2f p(k,j);
			//nextPts.push_back(p + (*uPtr).at<Point2f>(p));
		}
	}	
}

Mat of_driving::warpImage(Mat img, Mat mapping){
	Mat map_x, map_y, warped;
	map_x.create(img.size(), CV_32FC1);
	map_y.create(img.size(), CV_32FC1);

	for (int i = 0 ; i < img.rows ; i ++){
		for (int j = 0 ; j < img.cols ; j ++){
			Point2f p(j,i);
			map_x.at<float>(p) = j + mapping.at<Point2f>(p).x ;
			map_y.at<float>(p) = i + mapping.at<Point2f>(p).y ; 
		}
	}

	remap(img,warped,map_x,map_y, CV_INTER_LINEAR, BORDER_CONSTANT, Scalar(0,0,0));//*/

	return warped;
	
}


/********************************************************************************
	// EXAMPLE CODE FOR RESIZING THE OPTICAL FLOW FIELD TO THE NEXT LAYER 
	Mat dst, u_dst;
	resize(pyr[1],dst,Size(pyr[0].cols,pyr[0].rows));
	resize(optical_flow,u_dst,Size(pyr[0].cols,pyr[0].rows));

	Mat u_img;
	cvtColor(dst,u_img,CV_GRAY2BGR);
    
    for (int i = 0 ; i < dst.rows ; i += flowResolution){
 	   for (int j = 0 ; j < dst.cols ; j += flowResolution){
    	    Point2f p(j,i);
        	Point2f p2(p+u_dst.at<Point2f>(Point2f(j,i)));
        	arrowedLine2(u_img,p,p2,Scalar(0,0,255),1.5,8,0,0.1);
	    } 
	}


	// EXAMPLE CODE FOR REMAPPING
	Mat remapped;
	Mat map_x, map_y;
	map_x.create(dst.size(),CV_32FC1);
	map_y.create(dst.size(),CV_32FC1);

	for (int i = 0; i < dst.rows ; i ++){
		for (int j = 0 ; j < dst.cols ; j ++){
			map_x.at<float>(Point2f(j,i)) = j + u_dst.at<Point2f>(Point2f(j,i)).x;
			map_y.at<float>(Point2f(j,i)) = i + u_dst.at<Point2f>(Point2f(j,i)).y;
		}
	}

	remap(dst,remapped,map_x,map_y, CV_INTER_LINEAR, BORDER_CONSTANT, Scalar(0,0,0));

	imshow("resize test",dst);
	imshow("of resize test",u_img);
	imshow("remapped image",remapped);
/********************************************************************************/



/*********************************************************************************************/
/// HIERARCHICAL DOMINANT PLANE DETECTION ALGORITHM (Ohnishi, Imiya - Alg. 3)
/*for (int i = maxLayer ; i >= 0 ; i --){

	/// ---  1. Compute the optical flow field u(x,y,t) at layer l (output: optical_flow matrix)
	computeOpticalFlowField(*prevImgPtr,*ImgPtr);
	//optical_flow.copyTo(*uPtr);

	/// --- 2. Run the Dominant Plane Detection Algorithm - Output: Dominant Plane at layer l
	//DPD(*uPtr);

	if( i > 0){
		/// --- 3. Update the images on the next layer
		resize(*uPtr,*uPtr,u_pyr[i-1].size(),0,0,CV_INTER_AREA);
		if(LKpyr){
			updateInitialGuessOF();
		}
		*ImgPtr = warpImage(pyr[i-1],*uPtr);
		*prevImgPtr = prev_pyr[i-1];
		*uPtr = u_pyr[i-1];
	}

}//*/
/*********************************************************************************************/
