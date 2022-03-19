/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import frc.robot.Constants;
import frc.robot.Tunables;
import frc.robot.helpers.SubsystemInspector;
import frc.robot.helpers.DangerDetector;

public class Drivetrain extends SubsystemBase {

  private final SubsystemInspector inspector = new SubsystemInspector("Drivetrain");

  private final WPI_TalonFX leftFollower = new WPI_TalonFX(Constants.falconRearLeftCAN);
  private final WPI_TalonFX leftLeader = new WPI_TalonFX(Constants.falconFrontLeftCAN);

  // made the RearRight the leader, FrontRight Encoder is broken 
  private final WPI_TalonFX rightFollower = new WPI_TalonFX(Constants.falconFrontRightCAN);
  private final WPI_TalonFX rightLeader = new WPI_TalonFX(Constants.falconRearRightCAN);

  private final WPI_Pigeon2 pigeonGyro = new WPI_Pigeon2(Constants.pigeonCAN);

  private final DangerDetector dangerDetector = new DangerDetector();

  private SlewRateLimiter leftMetersPerSecondFilter = new SlewRateLimiter(Tunables.slewRateForDrivetrain.get());
  private SlewRateLimiter rightMetersPerSecondFilter = new SlewRateLimiter(Tunables.slewRateForDrivetrain.get());

  // TODO: figure out what these constants mean
  private final PIDController leftPIDController = new PIDController(Constants.kP, 0, Constants.kD);
  private final PIDController rightPIDController = new PIDController(Constants.kP, 0, Constants.kD);
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.kS, Constants.kV, Constants.kA);
  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
      Units.inchesToMeters(Constants.trackWidthInInches));

  private final DifferentialDriveOdometry odometry;
  private final Field2d field = new Field2d();

  public Drivetrain() {

    SmartDashboard.putData("Field", field);

    leftLeader.configFactoryDefault();
    leftLeader.setInverted(Constants.leftFalconsAreInverted);
    leftLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    leftLeader.configNeutralDeadband(Constants.drivetrainNeutralDeadbandPercentage);

    rightLeader.configFactoryDefault();
    rightLeader.setInverted(Constants.rightFalconsAreInverted);
    rightLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightLeader.configNeutralDeadband(Constants.drivetrainNeutralDeadbandPercentage);

    leftFollower.configFactoryDefault();
    leftFollower.follow(leftLeader);
    leftFollower.setInverted(InvertType.FollowMaster);

    rightFollower.configFactoryDefault();
    rightFollower.follow(rightLeader);
    rightFollower.setInverted(InvertType.FollowMaster);

    pigeonGyro.configFactoryDefault();
    pigeonGyro.reset();

    // TODO: pass in the initial position based on which autonomous routine we choose
    Pose2d initialPosition = new Pose2d(0.0, 0.0, getHeadingInRotation2d());

    // FIXME: need set an initial Pose based on which starting position we chose
    odometry = new DifferentialDriveOdometry(getHeadingInRotation2d());
    resetOdometry(initialPosition);
  }
  
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    double leftSensorUnitsPer100ms = leftLeader.getSelectedSensorVelocity();
    double rightSensorUnitsPer100ms = rightLeader.getSelectedSensorVelocity();
    double leftMetersPer100ms = convertSensorCountsToDistanceInMeters(leftSensorUnitsPer100ms);
    double rightMetersPer100ms = convertSensorCountsToDistanceInMeters(rightSensorUnitsPer100ms);
    double leftMetersPerSecond = leftMetersPer100ms * 10;
    double rightMetersPerSecond = rightMetersPer100ms * 10;
    return new DifferentialDriveWheelSpeeds(leftMetersPerSecond, rightMetersPerSecond);
  }

  public void resetOdometry(Pose2d pose) {
    leftLeader.setSelectedSensorPosition(0);
    rightLeader.setSelectedSensorPosition(0);
    odometry.resetPosition(pose, getHeadingInRotation2d());
  }

  public double getPitch() {
    double pitch = pigeonGyro.getPitch();
    return pitch;

  }
    
  public void driveWithThrottleAndSteering(double throttleMetersPerSecond, double steeringRotationRadiansPerSecond) {
    var wheelSpeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(throttleMetersPerSecond, 0.0, steeringRotationRadiansPerSecond));
    driveWithMetersPerSecond(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
  }

  public void driveWithMetersPerSecond(double leftTargetMetersPerSecond, double rightTargetMetersPerSecond) {

    // Adjust for tipping.
    leftTargetMetersPerSecond *= getTippingAdjustmentPercentage();
    rightTargetMetersPerSecond *= getTippingAdjustmentPercentage();

    double smoothedLeftMetersPerSecond = leftMetersPerSecondFilter.calculate(leftTargetMetersPerSecond);
    double leftFeedforward = feedforward.calculate(smoothedLeftMetersPerSecond);
    double leftActualSensorCountsPerSecond = leftLeader.getSelectedSensorVelocity();
    double leftActualMetersPerSecond = convertSensorCountsToDistanceInMeters(leftActualSensorCountsPerSecond);
    double leftFeedback = leftPIDController.calculate(leftActualMetersPerSecond, smoothedLeftMetersPerSecond);
    double leftVoltage = leftFeedback + leftFeedforward;
    leftLeader.setVoltage(leftVoltage);

    double smoothedRightMetersPerSecond = rightMetersPerSecondFilter.calculate(rightTargetMetersPerSecond);
    double rightFeedforward = feedforward.calculate(smoothedRightMetersPerSecond);
    double rightActualSensorCountsPerSecond = rightLeader.getSelectedSensorVelocity();
    double rightActualMetersPerSecond = convertSensorCountsToDistanceInMeters(rightActualSensorCountsPerSecond);
    double rightFeedback = rightPIDController.calculate(rightActualMetersPerSecond, smoothedRightMetersPerSecond);
    double rightVoltage = rightFeedback + rightFeedforward;
    rightLeader.setVoltage(rightVoltage);

    inspector.set("drive:leftTargetMetersPerSecond", leftTargetMetersPerSecond);
    inspector.set("drive:smoothedLeftMetersPerSecond", smoothedLeftMetersPerSecond);
    inspector.set("drive:rightTargetMetersPerSecond", rightTargetMetersPerSecond);
    inspector.set("drive:smoothedRightMetersPerSecond", smoothedRightMetersPerSecond);
  }

  public void driveWithPercentages(double leftPercent, double rightPercent) {
    leftLeader.set(leftPercent);
    rightLeader.set(rightPercent);
  }
  
  public void useBrakes() {
    leftLeader.setNeutralMode(NeutralMode.Brake);
    rightLeader.setNeutralMode(NeutralMode.Brake);
  }
  
  public void useCoast() {
    leftLeader.setNeutralMode(NeutralMode.Coast);
    rightLeader.setNeutralMode(NeutralMode.Coast);
  }

  @Override
  public void periodic() {

    // Allow tuning
    if (Tunables.slewRateForDrivetrain.didChange()) {
      leftMetersPerSecondFilter = new SlewRateLimiter(Tunables.slewRateForDrivetrain.get());
      rightMetersPerSecondFilter = new SlewRateLimiter(Tunables.slewRateForDrivetrain.get());
    }

    // Coast when disabled
    if (RobotState.isDisabled()) {
      leftLeader.setNeutralMode(NeutralMode.Coast);
      rightLeader.setNeutralMode(NeutralMode.Coast);
    } else {
      leftLeader.setNeutralMode(NeutralMode.Brake);
      rightLeader.setNeutralMode(NeutralMode.Brake);
    }

    // Watch tipping behavior.
    double pitch = pigeonGyro.getPitch();
    double roll = pigeonGyro.getRoll();
    dangerDetector.update(pitch, roll);

    // Update odometry
    Rotation2d heading = getHeadingInRotation2d();
    double leftDistanceInMeters = convertSensorCountsToDistanceInMeters(leftLeader.getSelectedSensorPosition());
    double rightDistanceInMeters = convertSensorCountsToDistanceInMeters(rightLeader.getSelectedSensorPosition());
    odometry.update(heading, -leftDistanceInMeters, -rightDistanceInMeters);

    // Show debug information in NetworkTables
    Pose2d pose = odometry.getPoseMeters();
    field.setRobotPose(pose);
    inspector.set("rotation", pose.getRotation().getDegrees());
    inspector.set("x", pose.getX());
    inspector.set("y", pose.getY());
    inspector.set("leftEncoder", leftLeader.getSelectedSensorPosition());
    inspector.set("rightEncoder", rightLeader.getSelectedSensorPosition());
    inspector.set("roll", pigeonGyro.getRoll());
    inspector.set("pitch", pigeonGyro.getPitch());
    inspector.set("yaw", pigeonGyro.getYaw());
    inspector.set("dangerLevel", dangerDetector.getDangerLevel());
    inspector.set("tippingAdjustmentPercentage", getTippingAdjustmentPercentage());
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftLeader.setVoltage(leftVolts);
    rightLeader.setVoltage(rightVolts);
  }
  
  public double getHeading() {
    return pigeonGyro.getRotation2d().getDegrees();
  }

  private Rotation2d getHeadingInRotation2d() {
    return pigeonGyro.getRotation2d();
  }

  private double convertSensorCountsToDistanceInMeters(double sensorCounts) {
    double motorRotations = (double) sensorCounts / Constants.sensorUnitsPerRevolution;
    double wheelRotations = motorRotations / Constants.driveGearRatio;
    double inchesOfRotation = wheelRotations * 2 * Math.PI * Constants.driveWheelRadiusInInches;
    return Units.inchesToMeters(inchesOfRotation);
  }

  private double getTippingAdjustmentPercentage() {
    if (Tunables.useAntiTipping.get() == false) {
      return 1.0;
    }
    double percentageAdjustment = 1.0;
    double dangerLevel = dangerDetector.getDangerLevel();
    double minDangerLevel = Tunables.minTippingIntegral.get();
    double maxDangerLevel = Tunables.maxTippingIntegral.get();
    if (dangerLevel > minDangerLevel) {
      percentageAdjustment = 1
          - (dangerLevel - minDangerLevel) / (maxDangerLevel - minDangerLevel);
    }
    return percentageAdjustment;
  }

  public Command getTrajectoryCommand() {
    // maybe?
    resetOdometry(new Pose2d(0, 0, getHeadingInRotation2d()));

    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        feedforward,
        kinematics,
        10);

    // Create config for trajectory
    double slowdownPercent = 0.22;
    TrajectoryConfig config = new TrajectoryConfig(
        slowdownPercent * Constants.maxSpeedInMetersPerSecond,
        slowdownPercent * Constants.maxAccelerationinMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(kinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(new Translation2d(10, 0)),
        new Pose2d(25, 0, new Rotation2d(0)),
        config);

    RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        this::getPose,
        new RamseteController(),
        feedforward,
        kinematics,
        this::getWheelSpeeds,
        leftPIDController,
        rightPIDController,
        // RamseteCommand passes volts to the callback
        this::tankDriveVolts,
        this);

    // Reset odometry to the starting pose of the trajectory.
    this.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> this.tankDriveVolts(0, 0));
  }

}
