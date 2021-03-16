// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.VecBuilder;
import frc.robot.commands.ResetDriveEncoders;
import frc.robot.models.BobTalonFX;
import frc.robot.models.LeaderBobTalonFX;
import frc.robot.models.PidGains;

public class Drivetrain extends SubsystemBase {
  private Field2d field = new Field2d();

  /** Constants **/
  public static final double wheelRadius = Units.inchesToMeters(3);
  public static final double wheelCircumference = (2 * Math.PI * wheelRadius);
  public static final double trackWidth = Units.inchesToMeters(24.5);
  public static final double gearRatio = 11.1111;
  public static final int encoderTicksPerRev = 2048;
  public static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(trackWidth);
  public static final double momentOfInertia = 7.5;
  public static final double massInKg = 60;
  // 10 is 10 units of 100ms
  public static final double conversionKonstant = wheelCircumference * 10 / (encoderTicksPerRev * gearRatio);

  public static final double ksVolts = 0.22;
  public static final double kvVoltSecondsPerMeter = 1.98;
  public static final double kaVoltSecondsSquaredPerMeter = 0.2;

  // kPDriveVel is the P value for tuning the control loop.
  public static final double kPDriveVel = 8.5;
  public static final double kMaxSpeedMetersPerSecon = 3; // 3
  public static final double kMaxAccelerationMetersPerSecondSquared = 3; // 3

  private static final PidGains leftLeadGains = new PidGains(0, 0, 0, 0, 0, 0);
  private static final PidGains rightLeadGains = new PidGains(0, 0, 0, 0, 0, 0);

  public static final double kRamseteB = 2;
  public static final double kRamseteZeta = 0.7;

  /** Inputs **/
  private double leftInput = 0;
  private double rightInput = 0;

  private final BobTalonFX leftFollow = new BobTalonFX(2);
  private final LeaderBobTalonFX leftLead = new LeaderBobTalonFX(1, leftFollow);
  private final TalonSRXSimCollection simSRX = new TalonSRXSimCollection(leftLead);

  private final BobTalonFX rightFollow = new BobTalonFX(4);
  private final LeaderBobTalonFX rightLead = new LeaderBobTalonFX(3, rightFollow);

  private final SpeedControllerGroup leftMotors = new SpeedControllerGroup(leftLead, leftFollow);

  // The motors on the right side of the drive.
  private final SpeedControllerGroup rightMotors = new SpeedControllerGroup(rightLead, rightFollow);

  // The robot's drive
  private final DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);

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
  private PigeonIMU pigeon = new PigeonIMU(5);

  public DifferentialDriveOdometry odometry;

  public DifferentialDrivetrainSim driveSim = new DifferentialDrivetrainSim(DCMotor.getFalcon500(2), gearRatio,
      momentOfInertia, massInKg, wheelRadius, trackWidth,

      // The standard deviations for measurement noise:
      // x and y: 0.001 m
      // heading: 0.001 rad
      // l and r velocity: 0.1 m/s
      // l and r position: 0.005 m
      VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

  /** Creates a new ExampleSubsystem. */
  public Drivetrain() {
    SmartDashboard.putData("Field", field);

    leftEncoder.setDistancePerPulse(2 * Math.PI * wheelRadius / encoderTicksPerRev);
    rightEncoder.setDistancePerPulse(2 * Math.PI * wheelRadius / encoderTicksPerRev);
    leftEncoder.reset();
    rightEncoder.reset();
    odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

    this.leftLead.setGains(leftLeadGains);
    this.rightLead.setGains(rightLeadGains);
  }

  // Set the inputs for the drivetrain
  public void SetInputs(double left, double right) {
    this.leftInput = left;
    this.rightInput = right;
  }

  public void ResetEncoders() {
    leftLead.setSelectedSensorPosition(0);
    rightLead.setSelectedSensorPosition(0);
  }

  // Reset the pose of the robot
  public void SetPose(Pose2d newPose) {
    this.leftEncoder.reset();
    this.rightEncoder.reset();
    this.gyro.reset();
    this.ResetEncoders();
    this.pigeon.setYaw(newPose.getRotation().getDegrees());
    this.odometry.resetPosition(newPose, gyro.getRotation2d());
  }

  public Pose2d GetPose() {
    return this.odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds GetWheelSpeeds() {
    double leftMetersPerSecond = leftLead.getSelectedSensorVelocity() / conversionKonstant;
    double rightMetersPerSecond = rightLead.getSelectedSensorVelocity() / conversionKonstant;
    return new DifferentialDriveWheelSpeeds(leftMetersPerSecond, rightMetersPerSecond);
  }

  public double getLeftPositionMeters() {
    return leftLead.getSelectedSensorPosition() / conversionKonstant;
  }

  public double getRightPositionMeters() {
    return rightLead.getSelectedSensorPosition() / conversionKonstant;
  }

  public void driveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(rightVolts);
    rightInput = rightVolts;
    leftInput = leftVolts;
    drive.feed();
  }

  public void driveMetersPerSecond(double leftMetersPerSecond, double rightMetersPerSecond) {
    this.leftLead.set(TalonFXControlMode.Velocity, leftMetersPerSecond * conversionKonstant);
    this.rightLead.set(TalonFXControlMode.Velocity, rightMetersPerSecond * conversionKonstant);
  }

  public void drivePercentOutput(double left, double right) {
    this.leftLead.set(TalonFXControlMode.PercentOutput, left);
    this.rightLead.set(TalonFXControlMode.PercentOutput, right);
  }

  @Override
  public void periodic() {
    odometry.update(gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
    field.setRobotPose(odometry.getPoseMeters());
    DifferentialDriveWheelSpeeds wheelSpeeds = GetWheelSpeeds();
    SmartDashboard.putNumber("Left m/s", wheelSpeeds.leftMetersPerSecond);
    SmartDashboard.putNumber("Right m/s", wheelSpeeds.rightMetersPerSecond);
    SmartDashboard.putNumber("Left m", getLeftPositionMeters());
    SmartDashboard.putNumber("Right m", getRightPositionMeters());

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
