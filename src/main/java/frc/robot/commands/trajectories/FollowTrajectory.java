// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.trajectories;

import edu.wpi.first.wpilibj.DriverStation;
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
    super(toFollow, Robot.drivetrain::getPose, new RamseteController(Drivetrain.kRamseteB, Drivetrain.kRamseteZeta),
        new SimpleMotorFeedforward(Drivetrain.ksVolts, Drivetrain.kvVoltSecondsPerMeter,
            Drivetrain.kaVoltSecondsSquaredPerMeter),
        Drivetrain.kinematics, Robot.drivetrain::getWheelSpeeds, new PIDController(Drivetrain.kPDriveVel, 0, 0),
        new PIDController(Drivetrain.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        Robot.drivetrain::tankDriveVolts, Robot.drivetrain);
  }
}
