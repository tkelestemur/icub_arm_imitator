# iCub simulation for arm imitation

Run in order:
1. `roscore`
2. YARP server: `yarp server --ros`
3. Run simulator: `iCub_SIM`
4. `simCartesianControl --robot <robot_name> --no_legs`
5. `iKinCartesianSolver --context simCartesianControl --part right_arm`
