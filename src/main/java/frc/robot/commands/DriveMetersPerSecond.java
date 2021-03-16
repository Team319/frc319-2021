// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class DriveMetersPerSecond extends CommandBase {
  /** Creates a new DriveMetersPerSecond. */
  private double leftMetersPerSecond = 0;
  private double rightMetersPerSecond = 0;

  public DriveMetersPerSecond(double leftMetersPerSecond, double rightMetersPerSecond) {
    addRequirements(Robot.drivetrain);
    this.leftMetersPerSecond = leftMetersPerSecond;
    this.rightMetersPerSecond = rightMetersPerSecond;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.drivetrain.driveMetersPerSecond(leftMetersPerSecond, rightMetersPerSecond);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Driving Robot");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.drivetrain.drivePercentOutput(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
