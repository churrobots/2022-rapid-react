/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import frc.robot.Constants;
import frc.robot.helpers.SubsystemInspector;
import frc.robot.helpers.SubsystemTuner.TunableBoolean;
import frc.robot.helpers.SubsystemTuner.TunableDouble;

public class Drivetrain extends SubsystemBase {

  private final SubsystemInspector inspector = new SubsystemInspector("Drivetrain");
  private final TunableBoolean useBrakes = new TunableBoolean("Drivetrain", "useBrakes", false);
  private final TunableDouble slewRate = new TunableDouble("Drivetrain", "slewRate", Constants.slewRateForDrivetrain);

  private final WPI_TalonFX leftFollower = new WPI_TalonFX(Constants.falconRearLeftCAN);
  private final WPI_TalonFX leftLeader = new WPI_TalonFX(Constants.falconFrontLeftCAN);

  // made the RearRight the leader, FrontRight Encoder is broken 
  private final WPI_TalonFX rightFollower = new WPI_TalonFX(Constants.falconFrontRightCAN);
  private final WPI_TalonFX rightLeader = new WPI_TalonFX(Constants.falconRearRightCAN);

  private final WPI_Pigeon2 pigeonGyro = new WPI_Pigeon2(Constants.pigeonCAN);

  private SlewRateLimiter leftMetersPerSecondFilter = new SlewRateLimiter(Constants.slewRateForDrivetrain);
  private SlewRateLimiter rightMetersPerSecondFilter = new SlewRateLimiter(Constants.slewRateForDrivetrain);

  // TODO: figure out what these constants mean
  private final PIDController leftPIDController = new PIDController(Constants.kP, 0, Constants.kD);
  private final PIDController rightPIDController = new PIDController(Constants.kP, 0, Constants.kD);
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.kS, Constants.kV, Constants.kA);
  // private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
  //     Units.inchesToMeters(Constants.trackWidthInInches));

  private final DifferentialDriveOdometry odometry;

  // TODO: revisit trajectory-following, and figure out where they got the "recommended" 2.0 and 0.7
  // https://github.com/wpilibsuite/allwpilib/tree/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/ramsetecontroller
  // private final RamseteController controller = new RamseteController(2.0, 0.7);

  public Drivetrain() {

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

    // FIXME: need set an initial Pose based on which starting position we chose
    odometry = new DifferentialDriveOdometry(getHeading());
    resetPosition();
  }

  public void resetPosition() {
    leftLeader.setSelectedSensorPosition(0);
    rightLeader.setSelectedSensorPosition(0);
    // TODO: figure out if we really should be resetting the pigeon here, the other team doesn't
    // https://github.com/FRC5190/FalconLibrary/blob/main/wpi/src/main/kotlin/org/ghrobotics/lib/subsystems/drive/FalconWestCoastDrivetrain.kt#L271
    pigeonGyro.reset();
    // TODO: take this as an argument (initial pose)
    Pose2d currentPosition = new Pose2d(0.0, 0.0, getHeading());
    odometry.resetPosition(currentPosition, getHeading());
  }

  public void driveWithMetersPerSecond(double leftTargetMetersPerSecond, double rightTargetMetersPerSecond) {

    // TODO: drive with speed + heading instead?
    // var wheelSpeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));

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
    inspector.set("drive:rightTargetMetersPerSecond", leftTargetMetersPerSecond);
    inspector.set("drive:leftVoltage", leftVoltage);
    inspector.set("drive:rightVoltage", rightVoltage);
  }

  public void driveWithPercentages(double leftPercent, double rightPercent) {
    leftLeader.set(leftPercent);
    rightLeader.set(rightPercent);    
  }

  @Override
  public void periodic() {
    // Allow tuning
    if (useBrakes.get()) {
      leftLeader.setNeutralMode(NeutralMode.Brake);
      rightLeader.setNeutralMode(NeutralMode.Brake);
    } else {
      leftLeader.setNeutralMode(NeutralMode.Coast);
      rightLeader.setNeutralMode(NeutralMode.Coast);
    }
    if (slewRate.didChange()) {
      leftMetersPerSecondFilter = new SlewRateLimiter(slewRate.get());
      rightMetersPerSecondFilter = new SlewRateLimiter(slewRate.get());
    }

    Rotation2d heading = getHeading();
    double leftDistanceInMeters = convertSensorCountsToDistanceInMeters(leftLeader.getSelectedSensorPosition());
    double rightDistanceInMeters = convertSensorCountsToDistanceInMeters(rightLeader.getSelectedSensorPosition());
    odometry.update(heading, leftDistanceInMeters, rightDistanceInMeters);
    // Show debug information in NetworkTables
    Pose2d pose = odometry.getPoseMeters();
    inspector.set("rotation", pose.getRotation().getDegrees());
    inspector.set("x", pose.getX());
    inspector.set("y", pose.getY());
    inspector.set("leftEncoder", leftLeader.getSelectedSensorPosition());
    inspector.set("rightEncoder", rightLeader.getSelectedSensorPosition());
    inspector.set("roll", pigeonGyro.getRoll());
    inspector.set("pitch", pigeonGyro.getPitch());
    inspector.set("yaw", pigeonGyro.getYaw());
  }

  private Rotation2d getHeading() {
    // TODO: confirm whether this needs to be negative for the unit circle nonsense
    return pigeonGyro.getRotation2d();
  }

  private double convertSensorCountsToDistanceInMeters(double sensorCounts) {
    double motorRotations = (double) sensorCounts / Constants.sensorUnitsPerRevolution;
    double wheelRotations = motorRotations / Constants.driveGearRatio;
    double inchesOfRotation = wheelRotations * 2 * Math.PI * Constants.driveWheelRadiusInInches;
    return Units.inchesToMeters(inchesOfRotation);
  }
  
}
