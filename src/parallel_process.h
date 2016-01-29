//Opencv includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "opencv2/nonfree/nonfree.hpp"

#include <iostream>



class ParallelDominantPlaneBuild : public cv::ParallelLoopBody{

    private:
    	int coreNum;
        cv::Mat& dp;
        cv::Mat& op;
        cv::Mat of;
        cv::Mat& pf;

        double epsilon;
        double Tc;
        double cut_f;
        double dilate_factor;
        double erode_factor;
    
    	cv::Matx22f A;
		cv::Matx21f b;


    public:
        ParallelDominantPlaneBuild(int ncores, cv::Mat& dpImage, cv::Mat& opImage, cv::Mat ofImage, cv::Mat& pfImage, double eps, double sampling_time, double f, double erode, double dilate, cv::Matx22f A_, cv::Matx21f b_)
                    : coreNum(ncores), dp(dpImage), op(opImage), of(ofImage), pf(pfImage), epsilon(eps), Tc(sampling_time), cut_f(f), erode_factor(erode), dilate_factor(dilate), A(A_), b(b_){}

        virtual void operator()(const cv::Range& range) const;
};



class ParallelGradientFieldBuild : public cv::ParallelLoopBody{

	private:
		int coreNum;
		cv::Mat& dp;
		cv::Mat& sp;
		cv::Mat& gf;
		double scale;
		double sigmaX;
		cv::Size size;
		int depth;

	public:
		ParallelGradientFieldBuild(int ncores, cv::Mat& dpImage, cv::Mat& spImage, cv::Mat& gfImage, double s, cv::Size size_, double sigma_x, int depth_) 
					: coreNum(ncores), dp(dpImage), sp(spImage), gf(gfImage), scale(s), size(size_), sigmaX(sigma_x), depth(depth_){}

		virtual void operator()(const cv::Range& range) const;

};


class ParallelOpticalFlow : public cv::ParallelLoopBody {

	private:
		int coreNum;
		double pyr_scale;
		double winSize;
		double maxLayer;
		double of_iterations;
		double poly_n;
		double poly_sigma;
		double flags;
		cv::Mat img, img2;
		/*cv::gpu::GpuMat img, img2;
		
		cv::gpu::FarnebackOpticalFlow& farneback_flow;
		const cv::gpu::GpuMat u_flow, v_flow;//*/
		cv::Mat& optical_flow;

	public:
		/*ParallelOpticalFlow(int cores, cv::gpu::FarnebackOpticalFlow& flowHandler, cv::gpu::GpuMat img_, cv::gpu::GpuMat img2_, const cv::gpu::GpuMat u, const cv::gpu::GpuMat v, cv::Mat& of)
					: coreNum(cores), farneback_flow(flowHandler), img(img_), img2(img2_), u_flow(u), v_flow(v), optical_flow(of){}//*/
		ParallelOpticalFlow(int cores, cv::Mat img_, cv::Mat img2_, cv::Mat& of, double pyr, double winsize, double levels, double iters, double poly_deg, double sigma, double flags_)
					: coreNum(cores), img(img_), img2(img2_), optical_flow(of), pyr_scale(pyr), winSize(winsize), maxLayer(levels), of_iterations(iters), poly_n(poly_deg), poly_sigma(sigma), flags(flags_){}//*/

		virtual void operator()(const cv::Range& range) const;

};



class ParallelDisplayImages : public cv::ParallelLoopBody {

	private:
		int coreNum;
		int flowResolution;
		cv::Mat img;
		cv::Mat of;
		cv::Mat pf;
		cv::Mat dp;
		cv::Mat sp;
		cv::Mat gf;
		cv::Matx21f p_bar;
		cv::Mat& total;
		cv::Rect dpROI;

	public:
		ParallelDisplayImages(int cores, int flow_res, cv::Mat img_,cv::Mat of_,cv::Mat pf_,cv::Mat dp_,cv::Mat sp_,cv::Mat gf_,cv::Matx21f p_bar_,cv::Mat& total_, cv::Rect roi)
					: coreNum(cores), flowResolution(flow_res), img(img_), of(of_), pf(pf_), dp(dp_), sp(sp_), gf(gf_), p_bar(p_bar_), total(total_), dpROI(roi){}
		virtual void operator()(const cv::Range& range) const;


};