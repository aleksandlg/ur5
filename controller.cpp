// File:          alex.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <iostream>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>

// All the webots classes are defined in the "webots" namespace
using namespace webots;

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  //const double target[3] = {-1, -0.5, 1};
  
  // get the time step of the current world.
  int timeStep = 32; //(int)robot->getBasicTimeStep();

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);
  Motor* motors[6];
  motors[0] = robot->getMotor("shoulder_pan_joint");
  motors[1] = robot->getMotor("shoulder_lift_joint");
  motors[2] = robot->getMotor("elbow_joint");
  
  motors[3] = robot->getMotor("wrist_1_joint");
  motors[4] = robot->getMotor("wrist_2_joint");
  motors[5] = robot->getMotor("wrist_3_joint");
  
  PositionSensor* senses[6];
  senses[0] = robot->getPositionSensor("shoulder_pan_joint_sensor");
  senses[1] = robot->getPositionSensor("shoulder_lift_joint_sensor");
  senses[2] = robot->getPositionSensor("elbow_joint_sensor");
  
  senses[3] = robot->getPositionSensor("wrist_1_joint_sensor");
  senses[4] = robot->getPositionSensor("wrist_2_joint_sensor");
  senses[5] = robot->getPositionSensor("wrist_3_joint_sensor");
  
  for(int i=0; i<6; ++i)
    senses[i]->enable(timeStep);
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();

    double val = senses[5]->getValue();
    std::cout<<val<<std::endl;
    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
    double theta[6] = {0.4829, -2.1102, 0, 1.3077, -1.5708, -2.0537}; 
    for(int i=0;i<6;++i)
      motors[i]->setPosition(theta[i]);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
