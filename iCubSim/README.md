# iCub simulation for arm imitation

Run in order:
1. YARP server: `yarp server`
2. Run simulator: `iCub_SIM` or `gazebo`
3. `simCartesianControl --robot <robot_name> --no_legs`
4. `iKinCartesianSolver --context simCartesianControl --part <part_name>`
