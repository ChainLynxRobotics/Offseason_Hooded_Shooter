// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.HoodedShooterSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

/** An example command that uses an example subsystem. */
public class ShootCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  //reference to the subsystem so that we can use it in the code
  private final HoodedShooterSubsystem m_subsystem;

  private double m_theta;
  private double m_velocity;

  private double m_targetX;
  private double m_targetY;

  /**
   * Creates a new ShootCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShootCommand(HoodedShooterSubsystem subsystem, double targetX, double targetY) {
    m_subsystem = subsystem;
    m_targetX = targetX;
    m_targetY = targetY;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  //call the CalculateAngleVelocity function from the subsystem
  @Override
  public void initialize() {}


  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  //TODO: add a timeout to the function so it ends after a certian amount of time
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
