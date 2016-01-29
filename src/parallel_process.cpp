#include "of_driving.h"


using namespace std;
using namespace cv;


void ParallelDominantPlaneBuild::operator()(const cv::Range& range) const{
     
    for(int k = range.start; k < range.end; k++)
    {

        cv::Mat dp_rect(dp,cv::Rect(0,dp.rows/coreNum*k,dp.cols,dp.rows/coreNum));
        cv::Mat op_rect(op,cv::Rect(0,op.rows/coreNum*k,op.cols,op.rows/coreNum));
        cv::Mat of_rect(of,cv::Rect(0,of.rows/coreNum*k,of.cols,of.rows/coreNum));
        cv::Mat pf_rect(pf,cv::Rect(0,pf.rows/coreNum*k,pf.cols,pf.rows/coreNum));
    
        for (int i = 0 ; i < pf_rect.rows ; i ++){
            Point2f* i_ptr = pf_rect.ptr<Point2f>(i);
            for (int j = 0 ; j < pf_rect.cols ; j ++){
                Matx21f p(j,i + k*dp.rows/coreNum);
                Matx21f planar_vec((A*p + b - p));
                Point2f planar_p(planar_vec(0),planar_vec(1));
                i_ptr[j] = planar_p;
            }
        }//*/

        int rows = dp_rect.rows;
        int cols = dp_rect.cols;
        
        if (dp_rect.isContinuous() && of_rect.isContinuous() && pf_rect.isContinuous()){
            cols *= rows;
            rows = 1;
        }//*/


        for (int i = 0 ; i < rows ; i ++){
            unsigned char * dp_ptr = dp_rect.ptr<uchar>(i);
            Point2f* of_ptr = of_rect.ptr<Point2f>(i);
            Point2f* pf_ptr = pf_rect.ptr<Point2f>(i);
            for (int j = 0 ; j < cols ; j ++){
                Point2f xdot(of_ptr[j]);
                Point2f xhat(pf_ptr[j]);
                
                if( norm(xdot - xhat) < epsilon ) {
                    dp_ptr[j] = 255;
                }
                else{
                    dp_ptr[j] = 0.0;
                }
            }
        }
        
        //cout << k << ": " << norm(mean(of_rect - pf_rect)) << endl; 

        /*for (int i = 0 ; i < rows ; i ++){
            unsigned char* dp_ptr = dp_rect.ptr<uchar>(i);
            unsigned char* op_ptr = op_rect.ptr<uchar>(i);
            for (int j = 0 ; j < cols ; j ++){
                    dp_ptr[j] = low_pass_filter(dp_ptr[j],op_ptr[j],Tc,1.0/cut_f);
            }
        }   

        dp_rect.copyTo(op_rect);

        double thresh = 100; //100
        double maxVal = 255;
        threshold(dp_rect,dp_rect,thresh,maxVal,THRESH_BINARY);//*/
    }//*/
}   

void ParallelGradientFieldBuild::operator()(const cv::Range& range) const {

    for (int k = range.start ; k < range.end ; k ++){

        cv::Mat dp_rect(dp,cv::Rect(0,dp.rows/coreNum*k,dp.cols,dp.rows/coreNum));
        cv::Mat sp_rect(sp,cv::Rect(0,sp.rows/coreNum*k,sp.cols,sp.rows/coreNum));
        cv::Mat gf_rect(gf,cv::Rect(0,gf.rows/coreNum*k,gf.cols,gf.rows/coreNum));//*/

        Size GaussSize(51,51);
        Mat grad_x, grad_y;
        double sigmaX = dp_rect.rows*dp_rect.cols*0.5;
        int ddepth = CV_32F; //CV_16S
        double delta = 0.0;

        GaussianBlur(dp_rect,sp_rect,GaussSize,sigmaX,0);
        
        Scharr(sp_rect, grad_x, ddepth, 1, 0, scale);
        Scharr(sp_rect, grad_y, ddepth, 0, 1, scale);
        

        int rows = dp_rect.rows;
        int cols = dp_rect.cols;

        if(grad_x.isContinuous() && grad_y.isContinuous() && gf_rect.isContinuous()){
            cols *= rows;
            rows = 1;
        }

        for (int i = 0 ; i < dp_rect.rows ; i ++){
            float* x_ptr = grad_x.ptr<float>(i);
            float* y_ptr = grad_y.ptr<float>(i);
            Point2f* grad_ptr = gf_rect.ptr<Point2f>(i);
            for (int j = 0 ; j < dp_rect.cols ; j ++){
                grad_ptr[j] = Point2f(x_ptr[j],y_ptr[j]);
            }
        }//*/

    }

}


void ParallelOpticalFlow::operator()(const cv::Range& range) const {

    for (int k = range.start ; k < range.end ; k ++){

        /*cv::Mat img_rect(img,cv::Rect(0,img.rows/coreNum*k,img.cols,img.rows/coreNum));
        cv::Mat img_rect2(img2,cv::Rect(0,img.rows/coreNum*k,img.cols,img.rows/coreNum));
        cv::Mat of_rect(optical_flow,cv::Rect(0,optical_flow.rows/coreNum*k,optical_flow.cols,optical_flow.rows/coreNum));//*/
        
        cv::Mat img_rect(img,cv::Rect(img.cols/coreNum*k,0,img.cols/coreNum,img.rows));
        cv::Mat img_rect2(img2,cv::Rect(img2.cols/coreNum*k,0,img2.cols/coreNum,img2.rows));
        cv::Mat of_rect(optical_flow,cv::Rect(optical_flow.cols/coreNum*k,0,optical_flow.cols/coreNum,optical_flow.rows));//*/
        
        calcOpticalFlowFarneback(img_rect, img_rect2, of_rect, pyr_scale, maxLayer, winSize, of_iterations, poly_n, poly_sigma, flags);//*/

        /*cv::gpu::GpuMat img_rect(img,cv::Rect(0,img.rows/coreNum*k,img.cols,img.rows/coreNum));
        cv::gpu::GpuMat img2_rect(img2,cv::Rect(0,img2.rows/coreNum*k,img2.cols,img2.rows/coreNum));
        cv::gpu::GpuMat u_rect(u_flow,cv::Rect(0,u_flow.rows/coreNum*k,u_flow.cols,u_flow.rows/coreNum));
        cv::gpu::GpuMat v_rect(v_flow,cv::Rect(0,v_flow.rows/coreNum*k,v_flow.cols,v_flow.rows/coreNum));
        cv::Mat of_rect(optical_flow,cv::Rect(0,optical_flow.rows/coreNum*k,optical_flow.cols,optical_flow.rows/coreNum));

        //cout << "img.rows/coreNum*k:" << img.rows/coreNum*k << endl;

        farneback_flow(img_rect,img2_rect,u_rect,v_rect);
        getFlowField(Mat(u_rect),Mat(v_rect),of_rect);//*/
    }

}


void ParallelDisplayImages::operator()(const cv::Range& range) const{

    for (int k = range.start ; k < 6 ; k ++){
        
        if(k == 0){
            Mat u_img, gImg;
            cvtColor(img,gImg,CV_BGR2GRAY);
            cvtColor(gImg,u_img,CV_GRAY2BGR);

            for (int i = 0 ; i < img.rows ; i+= flowResolution*2){
                const Point2f* of_ptr = of.ptr<Point2f>(i);
                for (int j = 0 ; j < img.cols ; j += flowResolution*2){
                    cv::Point2f p(j,i);
                    cv::Point2f p2(p + of_ptr[j]);
                    arrowedLine2(u_img,p,p2,Scalar(0,0,255),0.1,8,0,0.1); 
                }
            }

            u_img.copyTo(total(Rect(0,0,img.cols,img.rows)));
        }

        if(k == 1){
            Mat p_img, gImg;
            cvtColor(img,gImg,CV_BGR2GRAY);
            cvtColor(gImg,p_img,CV_GRAY2BGR);

            for (int i = 0 ; i < img.rows ; i+= flowResolution*2){
                const Point2f* pf_ptr = pf.ptr<Point2f>(i);
                for (int j = 0 ; j < img.cols ; j += flowResolution*2){
                    cv::Point2f p(j,i);
                    cv::Point2f p2(p + pf_ptr[j]);
                    arrowedLine2(p_img,p,p2,Scalar(255,255,0),0.1,8,0,0.1);
                }
            }
            p_img.copyTo(total(Rect(img.cols,0,img.cols,img.rows)));
        }        


        if(k == 2){
            Mat dp_img;
            cvtColor(dp,dp_img,CV_GRAY2BGR);
            rectangle(dp_img,dpROI.tl(),dpROI.br(),Scalar(0,255,0),4);
            dp_img.copyTo(total(Rect(2*img.cols,0,img.cols,img.rows)));
        }        

        if(k == 3){
            Mat sp_img;
            cvtColor(sp,sp_img,CV_GRAY2BGR);
            sp_img.copyTo(total(Rect(0,img.rows,img.cols,img.rows)));
        }       

        if(k == 4){
            Mat gradient_img;
            cvtColor(sp,gradient_img,CV_GRAY2BGR);

            for (int i = 0 ; i < img.rows ; i+= flowResolution*2){
                const Point2f* gf_ptr = gf.ptr<Point2f>(i);
                for (int j = 0 ; j < img.cols ; j += flowResolution*2){
                    cv::Point2f p(j,i);
                    cv::Point2f p2(p + gf_ptr[j]*0.1);
                    arrowedLine2(gradient_img,p,p2,Scalar(0,255,0),0.1,8,0,0.1);
                }
            }
            gradient_img.copyTo(total(Rect(img.cols,img.rows,img.cols,img.rows)));
        }        

        if(k == 5){
            Mat cf_img;
            img.copyTo(cf_img);
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

            arrowedLine2(cf_img,center,center + y*50,Scalar(255,0,0),3.0,8,0,0.1);
            arrowedLine2(cf_img,center,center + px,Scalar(0,green,red),3.0,8,0,0.1);
            cf_img.copyTo(total(Rect(2*img.cols,img.rows,img.cols,img.rows)));//*/

        }



    }

}

