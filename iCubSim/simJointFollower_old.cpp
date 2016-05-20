#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/PolyDriver.h>
#include <gsl/gsl_math.h>
#include <stdio.h>

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

void limitTorsoPitch(ICartesianControl *icart);



int main(int argc, char *argv[]) {

  Vector X_current, O_current;
  Vector X_desired, O_desired;


  Property option("(device cartesiancontrollerclient)");
  option.put("remote","/icubSim/cartesianController/right_arm");
  option.put("local","/cartesian_client/right_arm");
  PolyDriver clientCartCtrl(option);
  ICartesianControl *icart=NULL;
  if (clientCartCtrl.isValid()) {
     clientCartCtrl.view(icart);
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

  icart->getPose(X_current, O_current);
  fprintf(stdout,"Current Rotation (X)[m] = %s\n",X_current.toString().c_str());
  fprintf(stdout,"Current Orientation (O))[m] = %s\n",O_current.toString().c_str());

  X_desired.resize(3);
  O_desired.resize(4);

  X_desired = X_current;
  X_desired[0] += -0.1;
  O_desired = O_current;

  icart->goToPoseSync(X_desired,O_desired); // send request and wait for reply
  icart->waitMotionDone(0.04); // wait until the motion is done and ping at each 0.04 seconds

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
