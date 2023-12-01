// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HoodedShooterSubsystem extends SubsystemBase {

  private double targetAngle;
  private double targetRPM;

  private final CANSparkMax flywheelMotor;
  private final CANSparkMax hoodMotor;
  private final RelativeEncoder m_encoder1;
  private final RelativeEncoder m_encoder2;
  private final SparkMaxPIDController m_pidController1;
  private final SparkMaxPIDController m_pidController2;
  public static double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, flywheelSpeed, allowedErr;

  //this is the constructor, and is run when you create the subsystem
  //
  public HoodedShooterSubsystem() {

    //make motor for hood
    flywheelMotor = new CANSparkMax(1, MotorType.kBrushless);
    flywheelMotor.restoreFactoryDefaults();

    hoodMotor = new CANsparkMax(1, MotorType.kBrushless);
    hoodMotor.restoreFactoryDefaults();

    m_encoder1 = flywheelMotor.getEncoder();
    m_encoder1.setPosition(0);

    m_encoder2 = hoodMotor.getEncoder();
    m_encoder2.setPosition(0);

    m_pidController1 = flywheelMotor.getPIDcontroller();
    m_pidController2 = hoodMotor.getPIDcontroller();
    
    kP = 0.07; 
    kI = 1e-4;
    kD = 0.02; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    allowedErr = 0.1;
       
    m_pidController1.setP(kP);
    m_pidController1.setI(kI);
    m_pidController1.setD(kD);
    m_pidController1.setIZone(kIz);
    m_pidController1.setFF(kFF);
    m_pidController1.setOutputRange(kMinOutput, kMaxOutput);

    m_pidController2.setP(kP);
    m_pidController2.setI(kI);
    m_pidController2.setD(kD);
    m_pidController2.setIZone(kIz);
    m_pidController2.setFF(kFF);
    m_pidController2.setOutputRange(kMinOutput, kMaxOutput);
    
  }


  //this is the periodic function for the subsystem, and it run every tick
  //this is where the PID should be managed, and updated
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_pidController1.setReference(targetRPM, CANSparkMax.ControlType.kVelocity);
    m_pidController2.setReference(targetAngle, CANSparkMax.ControlType.kPosition);
  }

  //this is a funciton where you plug in the angle theta (in radians) to set the angle of the hood
  public void SetHoodAngle(double theta) {
    targetAngle = theta;
  }

  //this is where you plug in the rpm of the flywheel to set the speed
  public void SetFlywheelVelocity(double rpm) {
    targetRPM = rpm;
  }

  //calculates best angle for shooter to use
  public double calcudlateBestAngle(double targetX, double targetY) {
    double targetAngle = Math.atan(targetY/targetX);
    
    for(double angle = targetAngle; ; angle += 0.01 * Math.PI / 180) {
      double velocity = calculateAngleVelocity(targetX, targetY, angle);
      if(MpsToRPM(velocity) < Constants.maxFlywheelRPM) {
        return angle;
      }
    }
  }

  //calculates velocity for the shooter to use
  public double calculateAngleVelocity(double targetX, double targetY, double targetAngle) {
    double x = targetX; 
    double y = targetY; 
    double theta = targetAngle; 
    double g = Constants.gravity;
    // use our magical formula
    return Math.sqrt(
      -x*x*g / (
        (y - x * Math.tan(theta)) *
        (2 * Math.cos(theta) * Math.cos(theta))
      )
    );
  }
  public RelativeEncoder getFlywheelEncoder() {
    return m_encoder1;
  }
  public RelativeEncoder getHoodEncoder() {
    return m_encoder2;
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
