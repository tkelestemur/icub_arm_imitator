// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <stdio.h>
#include <yarp/os/all.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include "../include/sensor_msgs_JointState.h"

#include <string>

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;


int main(int argc, char const *argv[]) {
    Network yarp;

    Node node("/icubSim/state_publisher");

    Property options;
    options.put("device", "remote_controlboard");
    options.put("local", "/test/client");   //local port names
    options.put("remote", "/icubSim/torso");         //where we connect to

    // create a device
    PolyDriver robotDevice(options);
    if (!robotDevice.isValid()) {
        printf("Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        return 0;
    }

    IPositionControl *pos;
    IEncoders *encs;

    bool ok;
    ok = robotDevice.view(pos);
    ok = ok && robotDevice.view(encs);

    if (!ok) {
        printf("Problems acquiring interfaces\n");
        return 0;
    }


    int nj=0;
    pos->getAxes(&nj);
    Vector encoders;
    encoders.resize(nj);

    while(!encs->getEncoders(encoders.data()))
    {
        Time::delay(0.1);
        printf(".");
    }

    yarp::os::Publisher<sensor_msgs_JointState> joint_pub;
    if (!joint_pub.topic("/joint_states")) {
        std::cerr<< "Failed to create publisher to /chatter\n";
        return -1;
    }


    while (1) {

      encs->getEncoders(encoders.data());
      // std::cout << "raj0:" << encoders[0] << std::endl;

      sensor_msgs_JointState joint_states;

      joint_states.header.stamp.sec = Time::now();
      joint_states.name.resize(nj);
      joint_states.position.resize(nj);
      joint_states.name[0] ="j1";
      joint_states.position[0] = encoders[0];
      joint_states.name[1] ="j2";
      joint_states.position[1] = encoders[1];
      joint_states.name[2] ="j3";
      joint_states.position[2] = encoders[2];

      joint_pub.write(joint_states);



      Time::delay(0.2);
    }



  return 0;
}
