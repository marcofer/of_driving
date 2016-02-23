/**
 *
 * This example demonstrates how to get images from the robot remotely and how
 * to display them on your screen using opencv.
 *
 * Copyright Aldebaran Robotics
 */

// Aldebaran includes.
#include <alproxies/alvideodeviceproxy.h>
#include <alvision/alimage.h>
#include <alvision/alvisiondefinitions.h>
#include <alerror/alerror.h>
#include <alproxies/almotionproxy.h>
#include <alcommon/almodule.h>

// Opencv includes.
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <iostream>
#include <string>

#include "cJoystick.h"

#define TILT_JOINT "HeadPitch"
#define PAN_JOINT "HeadYaw"

#define X_BUTTON 2
#define SQUARE_BUTTON 3
#define TRIANGLE_BUTTON 0
#define CIRCLE_BUTTON 1
#define START_BUTTON 9

#define MINPITCH -0.671951
#define MAXPITCH 0.515047

using namespace AL;

void showImages(const std::string& robotIp)
{
  ALVideoDeviceProxy camProxy(robotIp, 9559);
  ALMotionProxy motion(robotIp,9559); //<--- Marco's code line
  ALMotionProxy* motionPtr = &motion;
  ALVideoDeviceProxy* cameraPtr;
  cameraPtr = &camProxy;

  std::string clientName;
  try{
  clientName = cameraPtr->subscribe("test", kQVGA, kBGRColorSpace, 30);
  }
  catch(...){
      cameraPtr->unsubscribe("test");
      clientName = cameraPtr->subscribe("test", kQVGA, kBGRColorSpace, 30);
  }

  cv::Mat imgHeader = cv::Mat(cv::Size(320, 240), CV_8UC3);


  double pan = 0.0;
  double tilt = 0.0;

  cJoystick joy;
  js_event* jse = joy.getJsEventPtr();
  joystick_state* js = joy.getJsStatePtr();

  motion.setStiffnesses("Body",1.0f);

  while (true)
  {

    if((char) cv::waitKey(30) == 27){
        break;
    }

    double dtilt;
    int res;

    res = joy.readEv(); //<--- don't print this value, or the joystick won't answer

    if(res != -1){
        std::cout << "res: " << res << std::endl;
        if(jse->type & JS_EVENT_BUTTON){
            if((int)jse->number == X_BUTTON){//tilt down
                dtilt = -1;
            }
            else if((int)jse->number == TRIANGLE_BUTTON){//tilt up
                dtilt = 1;
            }
            else{
                dtilt = 0;
            }
        }
    }

    double delta = 0.1;
    tilt += (double)dtilt*delta;

    tilt = (tilt < MAXPITCH) ? ((tilt > MINPITCH) ? (tilt) : (MINPITCH)) : (MAXPITCH) ;
    std::cout << "(1) --- tilt [rad]: " << tilt << "\t tilt [deg]: " << tilt/M_PI*180.0 << std::endl;
    dtilt = 0;

    AL::ALValue names = AL::ALValue::array(PAN_JOINT,TILT_JOINT);
    AL::ALValue angles = AL::ALValue::array(pan,tilt);
    try{
        motionPtr->post.setAngles(names,angles,0.1f);
    }
    catch(AL::ALError& err){
        std::cout << err.what() << std::endl;
        exit(1);
    }//*/
    ALValue img = cameraPtr->getImageRemote(clientName);

    imgHeader.data = (uchar*) img[6].getObject();

    cameraPtr->releaseImage(clientName);

    cv::imshow("images", imgHeader);
  }

  /** Cleanup.*/
  cameraPtr->unsubscribe(clientName);
}



int main(int argc, char* argv[])
{
  if (argc < 2)
  {
    std::cerr << "Usage 'getimages robotIp'" << std::endl;
    return 1;
  }

  const std::string robotIp(argv[1]);

  try
  {
    showImages(robotIp);
  }
  catch (const AL::ALError& e)
  {
    std::cerr << "Caught exception " << e.what() << std::endl;
  }

  return 0;
}

