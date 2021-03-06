// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

public class FollowTrajectory extends RamseteCommand {
  /** Creates a new FollowTrajectory. */
  public FollowTrajectory(Trajectory toFollow) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(toFollow, Robot.drivetrain::GetPose, new RamseteController(Drivetrain.kRamseteB, Drivetrain.kRamseteZeta),
        new SimpleMotorFeedforward(Drivetrain.ksVolts, Drivetrain.kvVoltSecondsPerMeter,
            Drivetrain.kaVoltSecondsSquaredPerMeter),
        Drivetrain.kinematics, Robot.drivetrain::GetWheelSpeeds, new PIDController(Drivetrain.kPDriveVel, 0, 0),
        new PIDController(Drivetrain.kPDriveVel, 0, 0), Robot.drivetrain::driveVolts, Robot.drivetrain);
  }
}
