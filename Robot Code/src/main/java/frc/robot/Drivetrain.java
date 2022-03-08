// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.SPI;
import frc.lib.swerveCode.SwerveModule;

import java.util.NoSuchElementException;
import java.util.OptionalDouble;

import com.kauailabs.navx.frc.*;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {

  public static final double kMaxSpeed = 0.1; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
  public static final double toDegrees = 180 / Math.PI;
  public static final double toRadians = Math.PI / 180;

  // Network Table instantiation
  private final NetworkTableInstance networkTable = NetworkTableInstance.getDefault();
  public NetworkTable ballAlignmentValues = networkTable.getTable("ballAlignment");

  // Bot measurements
  private final Translation2d frontLeftMotorLoc = new Translation2d(0.381, 0.381);
  private final Translation2d frontRightMotorLoc = new Translation2d(0.381, -0.381);
  private final Translation2d backLeftMotorLoc = new Translation2d(-0.381, 0.381);
  private final Translation2d backRightMotorLoc = new Translation2d(-0.381, -0.381);
  private static final DriverStation.Alliance alliance = DriverStation.getAlliance();

  // Swerve Module instantiation
  public final SwerveModule frontLeftMotor;
  public final SwerveModule frontRightMotor;
  public final SwerveModule backLeftMotor;
  public final SwerveModule backRightMotor;
  public final AHRS navX = new AHRS(SPI.Port.kMXP);
  private final Rotation2d initialMeasurement = new Rotation2d((navX.getYaw() % 360) * Math.PI / 180);

  // Shooter Range
  public double shooterRangeCm; // Enter shooter distance here (cm)
  public final Limelight limelight = new Limelight(61.49125);

  // Swerve drive library instantiation
  public final SwerveDriveKinematics drivetrainKinematics = new SwerveDriveKinematics(
      frontLeftMotorLoc, frontRightMotorLoc, backLeftMotorLoc, backRightMotorLoc);

  public final SwerveDriveOdometry drivetrainOdometry = new SwerveDriveOdometry(drivetrainKinematics,
      initialMeasurement);

  /**
   * Constructor for the swerve drivetrain
   * 
   * @param shooterRange
   *          The optimal shooting distance in cm for the robot (used for auto aligment with limelight)
   * @param swerveFrontLeftMotors
   *          The CAN ids for the swerve modules. The first element in the array should be the drive motor id, the
   *          second should be the turning motor id
   * @param swerveFrontRightMotors
   *          The CAN ids for the swerve modules. The first element in the array should be the drive motor id, the
   *          second should be the turning motor id
   * @param swerveBackLeftMotors
   *          The CAN ids for the swerve modules. The first element in the array should be the drive motor id, the
   *          second should be the turning motor id
   * @param swerveBackRightMotors
   *          The CAN ids for the swerve modules. The first element in the array should be the drive motor id, the
   *          second should be the turning motor id
   */
  public Drivetrain(double shooterRange, double[] swerveFrontLeftMotors, double[] swerveFrontRightMotors,
      double[] swerveBackLeftMotors, double[] swerveBackRightMotors) {
    navX.reset();
    shooterRangeCm = shooterRange;
    networkTable.startClientTeam(5109);
    frontLeftMotor = new SwerveModule((int) swerveFrontLeftMotors[0], (int) swerveFrontLeftMotors[1],
        (int) swerveFrontLeftMotors[2], swerveFrontLeftMotors[3]);
    frontRightMotor = new SwerveModule((int) swerveFrontRightMotors[0], (int) swerveFrontRightMotors[1],
        (int) swerveFrontRightMotors[2], swerveFrontRightMotors[3]);
    backLeftMotor = new SwerveModule((int) swerveBackLeftMotors[0], (int) swerveBackLeftMotors[1],
        (int) swerveBackLeftMotors[2], swerveBackLeftMotors[3]);
    backRightMotor = new SwerveModule((int) swerveBackRightMotors[0], (int) swerveBackRightMotors[1],
        (int) swerveBackRightMotors[2], swerveBackRightMotors[3]);
  }


  public void driveChassisSpeed(ChassisSpeeds wanted) {
    this.driveChassisSpeed(wanted, true);
  }

  public void driveChassisSpeed(ChassisSpeeds wanted, boolean fieldRelative) {
    Rotation2d navXVal = new Rotation2d((navX.getYaw() % 360) * Math.PI / 180);
    SwerveModuleState[] swerveModuleStates = drivetrainKinematics.toSwerveModuleStates(
      fieldRelative
          ? ChassisSpeeds.fromFieldRelativeSpeeds(wanted.vxMetersPerSecond, wanted.vyMetersPerSecond, wanted.omegaRadiansPerSecond, navXVal)
          : wanted
    );

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    frontLeftMotor.setDesiredState(swerveModuleStates[0]);
    frontRightMotor.setDesiredState(swerveModuleStates[1]);
    backLeftMotor.setDesiredState(swerveModuleStates[2]);
    backRightMotor.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed
   *          Speed of the robot in the x direction (forward).
   * @param ySpeed
   *          Speed of the robot in the y direction (sideways).
   * @param rot
   *          Angular rate of the robot.
   * @param fieldRelative
   *          Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    Rotation2d navXVal = new Rotation2d((navX.getYaw() % 360) * Math.PI / 180);
    SwerveModuleState[] swerveModuleStates = drivetrainKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, navXVal)
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    frontLeftMotor.setDesiredState(swerveModuleStates[0]);
    frontRightMotor.setDesiredState(swerveModuleStates[1]);
    backLeftMotor.setDesiredState(swerveModuleStates[2]);
    backRightMotor.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    drivetrainOdometry.update(
        new Rotation2d(navX.getYaw() * toDegrees),
        frontLeftMotor.getState(),
        frontRightMotor.getState(),
        backLeftMotor.getState(),
        backRightMotor.getState());
  }

  /** Limelight autoalign method */
  public void autoAlign() throws Throwable {

    try {
      // Calls limelight method to get the z-distance from goal
      OptionalDouble straightDistanceOption = limelight.calculateYDistance();
      double straightDistance = straightDistanceOption.getAsDouble();

      // Offset variable initialization
      double yOffset = straightDistance - shooterRangeCm;
      double xOffset = straightDistance * Math.tan(limelight.getXOffset().getAsDouble() * toRadians);

      // Control logic to drive bot into position
      if (Math.abs(yOffset) > .5 || Math.abs(xOffset) > .5) {
        drive(xOffset / Math.PI, yOffset / Math.PI, 0.0, true);
      }
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  /**
   * Initializes the listener for aligining to balls using the jetson nano program
   * Drives everytime the network tables is updated
   * 
   * @return Returns the id of the listener
   */
  public int initBallListener() {
    NetworkTableEntry allianceEntry = ballAlignmentValues.getEntry("alliance");
    allianceEntry.setString(alliance.toString());
    int listenerHandle = ballAlignmentValues.addEntryListener("tVelocity", (table, key, entry, value, flags) -> {
      double[] velocity = value.getDoubleArray();
      double xVel = velocity[0];
      double yVel = velocity[1];

      if (xVel != 0 && yVel != 0) {
        drive(xVel, yVel, 0, true);
      }
    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    return listenerHandle;
  }

  public int initShooterListener() {
    int listenerHandle = limelight.limelight.addEntryListener("tx", (table, key, entry, value, flags) -> {
      try {
        autoAlign();
      } catch (NoSuchElementException e) {
        System.out.println("Target not found");
      } catch (Throwable e) {
        e.printStackTrace();
      }
    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    return listenerHandle;
  }

  public void customAutoAlign() {
    double testCounter = 0;
    double degreeOffset = 0;

    double e_frontLeftPos = frontLeftMotor.m_turningEncoderAbsolute.getAbsolutePosition();
    double e_frontRightPos = frontRightMotor.m_turningEncoderAbsolute.getAbsolutePosition();
    double e_backRightPos = backRightMotor.m_turningEncoderAbsolute.getAbsolutePosition();
    double e_backLeftPos = backLeftMotor.m_turningEncoderAbsolute.getAbsolutePosition();

    while (testCounter == 0) {
      if (Math.abs(e_backLeftPos) > degreeOffset || Math.abs(e_frontLeftPos) > degreeOffset
          || Math.abs(e_frontRightPos) > degreeOffset || Math.abs(e_backRightPos) > degreeOffset) {
        testCounter = 1;
      }
      e_frontLeftPos = frontLeftMotor.m_turningEncoderAbsolute.getAbsolutePosition();
      e_frontRightPos = frontRightMotor.m_turningEncoderAbsolute.getAbsolutePosition();
      e_backRightPos = backRightMotor.m_turningEncoderAbsolute.getAbsolutePosition();
      e_backLeftPos = backLeftMotor.m_turningEncoderAbsolute.getAbsolutePosition();

      frontLeftMotor.m_turningMotor.set(-1 * e_frontLeftPos / 180);
      backLeftMotor.m_turningMotor.set(-1 * e_backLeftPos / 180);
      frontRightMotor.m_turningMotor.set(-1 * e_frontRightPos / 180);
      backRightMotor.m_turningMotor.set(-1 * e_backRightPos / 180);
    }
  }
}
