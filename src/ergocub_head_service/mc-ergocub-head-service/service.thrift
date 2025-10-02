 struct YarpMatrix 
 { 
 } 
 ( 
     yarp.name="yarp::sig::Matrix" 
     yarp.includefile="yarp/sig/Matrix.h" 
 ) 
service ergoCubHeadService
{
    bool rotateAxisAngle(1: double x, 2: double y, 3: double z, 4: double angle)
    bool setOrientation(1: YarpMatrix rot)
    bool setOrientationFlat(1: double r11, 2: double r12, 3: double r13, 4: double r21, 5: double r22, 6: double r23, 7: double r31, 8: double r32, 9: double r33)
    bool goHome();
    bool stop();
}