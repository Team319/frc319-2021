// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

public class FollowTrajExternalPID extends RamseteCommand {
  /** Creates a new FollowTrajExternalPID. */
  public FollowTrajExternalPID(Trajectory toFollow) {
    super(toFollow, Robot.drivetrain::GetPose, new RamseteController(Drivetrain.kRamseteB, Drivetrain.kRamseteZeta),
        Drivetrain.kinematics, Robot.drivetrain::driveMetersPerSecond, Robot.drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

}
