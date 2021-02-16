// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

public class FollowBarrelRacePath extends FollowPath {
  /** Creates a new FollowBarrelRacePath. */

  public FollowBarrelRacePath() {
    this(false);
  }

  public FollowBarrelRacePath(boolean isFirst) {
    super("BarrelRace", true, isFirst);
  }

}
