#include <signal.h>
#include "potentialNavigationNAO.h"
#include <alvision/alvisiondefinitions.h>
#include <alerror/alerror.h>
#include <pthread.h>
#include <qi/os.hpp>

#define LEFT_UD_AXIS 1
#define RIGHT_LR_AXIS 2
#define X_BUTTON 2
#define SQUARE_BUTTON 3
#define TRIANGLE_BUTTON 0
#define CIRCLE_BUTTON 1
#define START_BUTTON 9

#define TILT_JOINT "HeadPitch"
#define PAN_JOINT "HeadYaw"

#define MINPITCH -0.671951
#define MAXPITCH 0.515047
#define LINEAR_VEL_MAX 0.0952;
#define ANGULAR_VEL_MAX 0.83

#define SAVE_VIDEO 0



potentialNavigationNAO::potentialNavigationNAO(
    boost::shared_ptr<AL::ALBroker> broker, const string& name)
      : AL::ALModule(broker, name)
      , motionProxy(AL::ALMotionProxy(broker))
      , cameraProxy(AL::ALVideoDeviceProxy(broker))
{
  // Describe the module here. This will appear on the webpage
  setModuleDescription("Corridor Navigation Module.");
}


potentialNavigationNAO::~potentialNavigationNAO(){
    cout << "Destroyer called. " << endl ;
    cleanAllActivities();
}

void potentialNavigationNAO::init(){

    cout << "[potentialNavigationNAO] Initializing ... " << endl;

    online = true;

    ///Choose NAO camera
    camera_flag = 0 ;
    chooseCamera(camera_flag);

    /// call getImageRemote just once before the main loop to obtain NAOimage characteristics
    ALimg = cameraProxy.getImageRemote(cameraName);

    imgSize = Size((int)ALimg[0],(int)ALimg[1]);
    OCVimage.create(imgSize.height,imgSize.width,CV_8UC3);
    OCVimage.data = (uchar*)ALimg[6].GetBinary();
    img = Mat::zeros(imgSize,CV_8UC3);
    prev_img = Mat::zeros(imgSize,CV_8UC3);

    ///Joystick variables
    joy = new cJoystick;
    jse = joy->getJsEventPtr();
    js = joy->getJsStatePtr();

    ///Head pan-tilt variables
    pan = 0.0;
    tilt = 0.0;
    headset = false;
    firstKeyIgnored = false;

    /// of_driving object
    drive.set_imgSize(imgSize.width,imgSize.height);
    drive.initFlows();

    //Command variables
    manual = true;

    if(SAVE_VIDEO){
        enableRecording();
    }

    gettimeofday(&start_tod,NULL);
    elapsed_tod = 0.0;//*/

    run();

    return;

}


void potentialNavigationNAO::chooseCamera(int camera_flag){

    /// ALVideoDeviceProxy::subscribe(const std::string& vmName, const int& resolution, const int& colorSpace, const int& fps)
    /// vmName     – Name of the subscribing V.M.
    /// resolution – Resolution requested. { 0 = kQQVGA, 1 = kQVGA, 2 = kVGA, 3 = k4VGA }
    /// colorSpace – Colorspace requested. { 0 = kYuv, 9 = kYUV422, 10 = kYUV, 11 = kRGB, 12 = kHSY, 13 = kBGR }
    /// fps        – Fps (frames per second) requested to the video source.

    cameraName = (camera_flag) ? ("bottom") : ("top");
    cameraName = cameraProxy.subscribeCamera(cameraName, camera_flag, kQVGA, kBGRColorSpace, 30);

    /*try{
        cameraName = cameraProxy.subscribeCamera(cameraName, camera_flag, kQVGA, kBGRColorSpace, 30);

    }
    catch(...){
        cameraProxy.unsubscribe(cameraName);
        cameraName = cameraProxy.subscribeCamera(cameraName, camera_flag, kQVGA, kBGRColorSpace, 30);
    }//*/

    cout << "Camera chosen: " << cameraName << endl;
}




void potentialNavigationNAO::cleanAllActivities(){
    delete joy;
    cameraProxy.unsubscribe(cameraName);
    if(SAVE_VIDEO){
        recorderProxy.stopRecording();
    }
    cv::destroyAllWindows();
    cout << "all active works have been stopped. " << endl ;

}


void potentialNavigationNAO::setTiltHead(){

    int dtilt = 0;
    int res;

    res = joy->readEv(); //<--- don't print this value, or the joystick won't answer

    if(res != -1){

        if(jse->type & JS_EVENT_BUTTON){
            if((int)jse->number == X_BUTTON){//tilt down
                dtilt = -1;
            }
            else if((int)jse->number == TRIANGLE_BUTTON){//tilt up
                dtilt = 1;
            }
            else if((int)jse->number == START_BUTTON){//tilt down
                (firstKeyIgnored) ? (headset = true) : (firstKeyIgnored = true) ; //<-- needed to avoid the first automatic push of all buttons of joystick at initialization
            }
            else{
                dtilt = 0;
            }
        }

        updateTilt(dtilt);

    }


    cout << "(1) --- tilt [rad]: " << tilt << "\t tilt [deg]: " << tilt/M_PI*180.0 << endl;


    AL::ALValue names = AL::ALValue::array(PAN_JOINT,TILT_JOINT);
    AL::ALValue angles = AL::ALValue::array(pan,tilt);
    double fracSpeed = 0.2;

    //motionProxy.angleInterpolation(names,angles,1.0f,true);



    int id;
    try{
    id = motionProxy.post.setAngles(names,angles,fracSpeed);
    motionProxy.wait(id,1000);
    }
    catch(AL::ALError& err){
        cout << err.what() << endl;
        std::exit(1);
    }//*/

    //capture image from subscribed camera
    ALimg = cameraProxy.getImageRemote(cameraName);
    imshow("camera",OCVimage);
    waitKey(1);

}


void potentialNavigationNAO::enableRecording(){
    string folder_path = "/home/nao/recordings/cameras/NAO_potentialNavigation/video";

    recorderProxy.setColorSpace(11);//AL::kRGBColorSpace : buffer contains triplet on the format 0xBBGGRR, equivalent to three unsigned char
    recorderProxy.setResolution(1);//kQVGA
    recorderProxy.setVideoFormat("MJPG");
    recorderProxy.setFrameRate(30);
    recorderProxy.startRecording(folder_path,"NAOvideo",true);//*/
}

void potentialNavigationNAO::updateTilt(int dtilt){
    double delta = 0.1;
    tilt += (double)dtilt*delta;

    tilt = (tilt < MAXPITCH) ? ((tilt > MINPITCH) ? (tilt) : (MINPITCH)) : (MAXPITCH) ;
}

void potentialNavigationNAO::run(){

    char key;
    cout << "Running ... " << endl;

    /*if(0){
      cout << "enable foot contact protection" << endl;
      ALValue aux;
      aux.arrayPush("ENABLE_FOOT_CONTACT_PROTECTION");
      aux.arrayPush(false);
      config.arrayPush(aux);
      motionProxy.setMotionConfig(config);
    }//*/

    motionProxy.stiffnessInterpolation("Body", 1.0, 1.0); //// DOES THE STIFFNESS HAVE TO BE  1.0?????
    qi::os::sleep(2);
    motionProxy.moveInit();


    while(true){

        key = cv::waitKey(30);

        if((int)key == 27){
          cout << "[potentialNavigationNAO] Caught ESC!"<< endl;
          break;
        }

        if(online){
            if(!headset){
                setTiltHead();
                //drive.createWindowAndTracks();
            }
            else{

                // tic ...
                gettimeofday(&tic,NULL);

                //capture image from subscribed camera
                ALimg = cameraProxy.getImageRemote(cameraName);

                //update working images
                img.copyTo(prev_img);// = img.clone();
                OCVimage.copyTo(img);// = imgHeader.clone();

                imshow("second image", OCVimage);
                waitKey(30);
                /*Mat* heapImg = new Mat;
                Mat* heapPrevImg = new Mat;
                *heapImg = img;
                *heapPrevImg = prev_img;

                //make the algorithm run
                updateTcAndLowPass();

                try{
                    //drive.run(img,prev_img);
                    drive.run(*heapImg,*heapPrevImg);
                }
                catch(...){
                    cerr << "Problem in drive.run. " << endl;
                    std::exit(1);
                }

                //if manual then read joystick values
                double v = drive.get_linearVel();
                double wnorm = drive.get_angularVel();

                double w = wnorm*ANGULAR_VEL_MAX;

                //command NAO

                cout << "v: " << v << endl;
                cout << "w: " << w << endl;

                motionProxy.move(v,0.0f,-w);


                // ... toc
                gettimeofday(&toc,NULL);

                //measure the interleaving time
                double tictoc = (toc.tv_sec + (double)toc.tv_usec /1000000.0)
                      - (tic.tv_sec + (double)tic.tv_usec/1000000.0);

                //sleep to make the code run at 30 Hz
                if(0.0333 - tictoc > 0)
                    sleep(0.0333 - tictoc);

                delete heapImg;
                delete heapPrevImg;
                checkForSIGINT();//*/
            }
        }

    }

    cleanAllActivities();

}

void potentialNavigationNAO::updateTcAndLowPass(){
    gettimeofday(&end_tod,NULL);

    elapsed_tod = (end_tod.tv_sec + (double)end_tod.tv_usec /1000000.0)
          - (start_tod.tv_sec + (double)start_tod.tv_usec/1000000.0);

    start_tod = end_tod;
    double fc_ratio = 0.25;
    double cutoff_f = 1.0/elapsed_tod*fc_ratio;

    drive.setTc(elapsed_tod);
    drive.setImgLowPassFrequency(cutoff_f);
    drive.setBarLowPassFrequency(cutoff_f*fc_ratio*0.5);
}
