
#include <stdio.h>
#include <iostream>

#include <yarp/os/all.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>

#include "../include/geometry_msgs_Pose.h"

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace std;


void limitTorsoPitch();

int main(int argc, char const *argv[]) {
  PolyDriver dd;
  ICartesianControl *icart;
  Vector cRot, cOrt; // current rotation and orientation of robot's joint
  Vector dRot, dOrt; // desired rotation and orientation of robot's joint

  printf("ControlThread:starting\n");

  Property options("(device cartesiancontrollerclient)");
  options.put("remote","/icubSim/cartesianController/right_arm");
  options.put("local","/cartesian_client/right_arm");

  dd.open(options);

  if (dd.isValid()) {
     dd.view(icart);

     if (!icart){
        return false;
     }
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
  // limitTorsoPitch();

  // send the request for dofs reconfiguration
  icart->setDOF(newDof,curDof);

  icart->setTrajTime(1.0);

  dRot.resize(3);
  dOrt.resize(4);





  Network yarp;
  Node node("/icubSim/poseSub");
  yarp::os::Subscriber<geometry_msgs_Pose>  poseSub;
  poseSub.topic("/icub/jointPose");



  return 0;
}

// void limitTorsoPitch()
// {
//     int axis=0; // pitch joint
//     double min, max;
//     int MAX_TORSO_PITCH = 30.0;
//     // sometimes it may be helpful to reduce
//     // the range of variability of the joints;
//     // for example here we don't want the torso
//     // to lean out more than 30 degrees forward
//
//     // we keep the lower limit
//     icart->getLimits(axis,&min,&max);
//     icart->setLimits(axis,min,MAX_TORSO_PITCH);
// }
