

#include "geometry_msgs_Pose.h"
#include <stdio.h>
#include <yarp/os/all.h>

using namespace yarp::os;
#include <iostream>
using namespace std;

int main(int argc, char *argv[]) {
    Network yarp;
    Node node("/icubSim/poseSub");
    yarp::os::Subscriber<geometry_msgs_Pose>  poseSub;

    poseSub.topic("/icub/jointPose");

    int count = 0;
    while (true) {

        geometry_msgs_Pose* jointPose  = poseSub.read();
        cout << "x position: " << jointPose->position.x  <<endl;
        // printf("Got [%d]\n",jointPose.position.x);

    }

    return 0;
}
