// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.VecBuilder;

public class Drivetrain extends SubsystemBase {
  private Field2d field = new Field2d();

  /** Constants **/
  public double wheelRaidus = Units.inchesToMeters(3);
  public double trackWidth = Units.inchesToMeters(24.5);
  public int encoderTicksPerRev = 1024;
  public DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(trackWidth);

  /** Inputs **/
  private double leftInput = 0;
  private double rightInput = 0;

  // These represent our regular encoder objects, which we would
  // create to use on a real robot.
  private Encoder leftEncoder = new Encoder(0, 1);
  private Encoder rightEncoder = new Encoder(2, 3);

  // These are our EncoderSim objects, which we will only use in
  // simulation. However, you do not need to comment out these
  // declarations when you are deploying code to the roboRIO.
  private EncoderSim leftEncoderSim = new EncoderSim(leftEncoder);
  private EncoderSim rightEncoderSim = new EncoderSim(rightEncoder);

  // Create our gyro object like we would on a real robot.
  private AnalogGyro gyro = new AnalogGyro(1);

  // Create the simulated gyro object, used for setting the gyro
  // angle. Like EncoderSim, this does not need to be commented out
  // when deploying code to the roboRIO.
  private AnalogGyroSim gyroSim = new AnalogGyroSim(gyro);

  public DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

  public DifferentialDrivetrainSim driveSim = new DifferentialDrivetrainSim(DCMotor.getFalcon500(2), 11.1111, 7.5, 60,
      Units.inchesToMeters(3), Units.inchesToMeters(24.5),

      // The standard deviations for measurement noise:
      // x and y: 0.001 m
      // heading: 0.001 rad
      // l and r velocity: 0.1 m/s
      // l and r position: 0.005 m
      VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

  /** Creates a new ExampleSubsystem. */
  public Drivetrain() {
    SmartDashboard.putData("Field", field);

    leftEncoder.setDistancePerPulse(2 * Math.PI * wheelRaidus / encoderTicksPerRev);
    rightEncoder.setDistancePerPulse(2 * Math.PI * wheelRaidus / encoderTicksPerRev);
  }

  // Set the inputs for the drivetrain
  public void SetInputs(double left, double right) {
    this.leftInput = left;
    this.rightInput = right;
  }

  // Reset the pose of the robot
  public void SetPose(Pose2d newPose) {
    this.leftEncoder.reset();
    this.rightEncoder.reset();
    this.gyro.reset();
    this.odometry.resetPosition(newPose, gyro.getRotation2d());
  }

  @Override
  public void periodic() {
    odometry.update(gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
    field.setRobotPose(odometry.getPoseMeters());
  }

  @Override
  public void simulationPeriodic() {
    // Set the inputs to the system. Note that we need to convert
    // the [-1, 1] PWM signal to voltage by multiplying it by the
    // robot controller voltage.
    driveSim.setInputs(this.leftInput * RobotController.getInputVoltage(),
        this.rightInput * RobotController.getInputVoltage());

    // Advance the model by 20 ms. Note that if you are running this
    // subsystem in a separate thread or have changed the nominal timestep
    // of TimedRobot, this value needs to match it.
    driveSim.update(0.02);

    // Update all of our sensors.
    leftEncoderSim.setDistance(driveSim.getLeftPositionMeters());
    leftEncoderSim.setRate(driveSim.getLeftVelocityMetersPerSecond());
    rightEncoderSim.setDistance(driveSim.getRightPositionMeters());
    rightEncoderSim.setRate(driveSim.getRightVelocityMetersPerSecond());
    gyroSim.setAngle(-driveSim.getHeading().getDegrees());
  }
}
