#ifndef CONSTANT_H
#define CONSTANT_H

const int NUMBER_CAR = 2;
const int NUM_WHEEL = 4;
const int MAX_NUM_HANDS = 1;
const float HAND_BOX_CM = 20; // 25 cm
//const float HAND_BOX_CM = 40;
//const float MIN_HAND_PIX = 29; 
//const int HAND_GRID_SIZE = 841;
//const float KINECT_PIX_PER_DEPTH = 0.8982;
const float KINECT_PIX_PER_DEPTH = 0.67;
const int GRID_SIZE =19200;

#define CV_RED cvScalar(255,0,0)
#define CV_GREEN cvScalar(0,255,0)
#define CV_BLUE cvScalar(0,0,255)
#define CV_WHITE cvScalar(255,255,255)

const float SPHERE_SIZE = 2;
const float CUBE_SIZE = 4;
//#define TRACKING_SIZE cvSize(160,120)

const int		SKIN_X = 320;
const int		SKIN_Y = 240;
const float MIN_HAND_PIX = 21; // 11 pixels
const int		HAND_GRID_SIZE = 441;// 15x15
const int		HAND_SIZE = HAND_GRID_SIZE;

const int HAND_MAX_TRANSMIT_SIZE = 120;

//for file loading
#define MARKER_FILENAME "Data/Celica.bmp"
//#define MARKER_FILENAME "Data/thai_art.jpg"
#define CAMERA_PARAMS_FILENAME "Data/Cameras/camera.yml"
#define KINECT_PARAMS_FILENAME "Data/Cameras/kinect.yml"
#define KINECT_TRANSFORM_FILENAME "Data/Cameras/KinectTransform.yml"
#define KINECT_CONFIG_FILENAME "Data/Cameras/KinectConfig.xml"

#define CAR1_BODY_FILENAME "Data/Cars/GT4_body.ive"
#define CAR1_WHEEL_FILENAME "Data/Cars/GT4_tire.ive"
#define CAR2_BODY_FILENAME "Data/Cars/Murcielago_body.ive"
#define CAR2_WHEEL_FILENAME "Data/Cars/Murcielago_tire.ive"

//for image processing
#define CAPTURE_SIZE cvSize(640,480)
#define REGISTRATION_SIZE cvSize(320,240)
#define MESH_SIZE cvSize(160,120)
#define OPFLOW_SIZE cvSize(SKIN_X,SKIN_X)
#define SKIN_SEGM_SIZE cvSize(SKIN_X,SKIN_Y)

//for VRPN connection
const int UDP_LIMITATION = 100;

//const float MIN_HAND_PIX = 15; // 11 pixels
//const int HAND_GRID_SIZE = 225;// 15x15

#endif