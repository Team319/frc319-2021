// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FollowPath extends SequentialCommandGroup {

  Trajectory trajectory = new Trajectory();

  public FollowPath(String pathName) {
    this(pathName, true, false);
  }

  public FollowPath(String pathName, boolean stopAtEnd) {
    this(pathName, stopAtEnd, false);
  }

  /** Creates a new FollowPath. */
  public FollowPath(String pathName, boolean stopAtEnd, boolean setOdometry) {
    String trajectoryJSON = "trajectories/output/" + pathName + ".wpilib.json";
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      if (setOdometry) {
        addCommands(new SetOdometry(trajectory.getInitialPose()));
      }
      addCommands(new FollowTrajExternalPID(trajectory));
      if (stopAtEnd) {
        addCommands(new StopDrivetrain());
      }
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
  }
}
