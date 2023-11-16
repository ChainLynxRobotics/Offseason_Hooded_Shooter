// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HoodedShooterSubsystem extends SubsystemBase {

  private double targetAngle;
  private double targetRPM;

  //this is the constructor, and is run when you create the subsystem
  //
  public HoodedShooterSubsystem() {}


  //this is the periodic function for the subsystem, and it run every tick
  //this is where the PID should be managed, and updated
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //this is a funciton where you plug in the angle theta (in radians) to set the angle of the hood
  public void SetHoodAngle(double theta) {
    targetAngle = theta;
  }

  //this is where you plug in the rpm of the flywheel to set the speed
  public void SetFlywheelVelocity(double rpm) {
    targetRPM = rpm;
  }

  //TODO: this is where you should calculate the ideal angle and velocity for the shooter to use
  public void calculateAngleVelocity() {

  }

  //helper function to convert meters per second to rotations per minute
  public double MpsToRPM(double mps) {
    //Convert from mps to angular velocity (radians / seconds)
    //v = wr
    double angularVel = mps/Constants.flywheelRad;

    //convert from angular velocity to rpm
    double rpm = angularVel * 60 / (Math.PI * 2);

    return rpm;
  }
}
