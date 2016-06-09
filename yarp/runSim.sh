#!/bin/bash
# My first script

echo "Running iCub Sim!"

iCub_SIM && simCartesianControl --no_legs && iKinCartesianSolver --context simCartesianControl --part right_arm
