// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/os/Property.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>

#include <string>
#include <iostream>

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;

using namespace std;

class ControlThread: public RateThread
{

protected:
    PolyDriver dd;
    ICartesianControl *icart;
    IVelocityControl *ivel;

    Vector X_current, O_current;
    Vector X_desired, O_desired;

public:
    ControlThread(int period):RateThread(period){}

    bool threadInit()
    {
        //initialize here variables
        printf("ControlThread:starting\n");

        Property options("(device cartesiancontrollerclient)");
        options.put("remote","/icubSim/cartesianController/right_arm");
        options.put("local","/cartesian_client/right_arm");

        dd.open(options);

        if (dd.isValid()) {
           dd.view(icart);
           std::cout << "driver is available!" << std::endl; // debug
        }


        // get the torso dofs
        Vector newDof, curDof;
        icart->getDOF(curDof);
        newDof=curDof;

        // enable the torso yaw and pitch
        // disable the torso roll
        newDof[0]=0;
        newDof[1]=0;
        newDof[2]=0;

        // impose some restriction on the torso pitch
        limitTorsoPitch(icart);

        // send the request for dofs reconfiguration
        icart->setDOF(newDof,curDof);

        icart->setTrajTime(1.0);

        X_desired.resize(3);
        O_desired.resize(4);

        return true;
    }

    void threadRelease()
    {
        printf("ControlThread:stopping the robot\n");

        ivel->stop();

        dd.close();

        printf("Done, goodbye from ControlThread\n");
    }

    void run()
    {

      std::cout << "runned!" << std::endl; // debug
      X_desired = X_current;
      X_desired[0] += -0.01;
      O_desired = O_current;

      // icart->goToPoseSync(X_desired,O_desired); // send request and wait for reply
      // icart->waitMotionDone(0.04); // wait until the motion is done and ping at each 0.04 seconds
    }


    void limitTorsoPitch(ICartesianControl *icart)
    {
        int axis=0; // pitch joint
        double min, max;
        int MAX_TORSO_PITCH = 30.0;
        // sometimes it may be helpful to reduce
        // the range of variability of the joints;
        // for example here we don't want the torso
        // to lean out more than 30 degrees forward

        // we keep the lower limit
        icart->getLimits(axis,&min,&max);
        icart->setLimits(axis,min,MAX_TORSO_PITCH);
    }

    void printPoseStatus(){
      icart->getPose(X_current, O_current);
      std::cout << "Current Rotation (X)[m] = " << X_current.toString().c_str() << std::endl;
      std::cout << "Current Orientation (O)[m] = " << O_current.toString().c_str() << std::endl;

      // fprintf(stdout,"Current Rotation (X)[m] = %s\n",X_current.toString().c_str());
      // fprintf(stdout,"Current Orientation (O))[m] = %s\n",O_current.toString().c_str());


    }


};

int main(int argc, char *argv[])
{
    Network yarp;

    if (!yarp.checkNetwork())
    {
        printf("No yarp network, quitting\n");
        return 1;
    }


    ControlThread ctrlThread(4000); //period is 40ms

    ctrlThread.start();
    // int RUN_TIME = 3; // seconds
    // bool done=false;
    // double startTime=Time::now();
    // while(!done)
    // {
    //     if ((Time::now()-startTime)>RUN_TIME)
    //         done=true;
    // }

    ctrlThread.stop();

    return 0;
}
