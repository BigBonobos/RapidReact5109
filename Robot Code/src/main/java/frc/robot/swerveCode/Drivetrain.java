// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerveCode;


import frc.robot.Limelight;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort.Port;

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

  // Network Table instantiation
  private final NetworkTableInstance ntwrkInst = NetworkTableInstance.getDefault();
  public NetworkTable ballAlignmentValues = ntwrkInst.getTable("ballAlignment");

  // Bot measurements
  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);
  private static final DriverStation.Alliance alliance = DriverStation.getAlliance();

  // Swerve Module instantiation
  public SwerveModule m_frontLeft;
  public SwerveModule m_frontRight;
  public SwerveModule m_backLeft;
  public SwerveModule m_backRight;
  private AHRS navX = new AHRS(Port.kUSB);
  private Rotation2d initialMeasurement = new Rotation2d((navX.getYaw() % 360) * Math.PI/180);

  // Shooter Range
  public double shooterRangeCm; // Enter shooter distance here (cm)
  public final Limelight limelight = new Limelight(61.49125);

  // Swerve drive library instantiation
  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(m_kinematics, initialMeasurement);


  /**
   * Constructor for the swerve drivetrain
   * @param shooterRange The optimal shooting distance in cm for the robot (used for auto aligment with limelight)
   * @param swerveFrontLeftMotors  The CAN ids for the swerve modules. The first element in the array should be the drive motor id, the second should be the turning motor id
   * @param swerveFrontRightMotors  The CAN ids for the swerve modules. The first element in the array should be the drive motor id, the second should be the turning motor id
   * @param swerveBackLeftMotors  The CAN ids for the swerve modules. The first element in the array should be the drive motor id, the second should be the turning motor id
   * @param swerveBackRightMotors The CAN ids for the swerve modules. The first element in the array should be the drive motor id, the second should be the turning motor id
   */
  public Drivetrain(double shooterRange, double[] swerveFrontLeftMotors, double[] swerveFrontRightMotors, double[] swerveBackLeftMotors, double[] swerveBackRightMotors) {
    navX.reset();
    shooterRangeCm = shooterRange;
    ntwrkInst.startClientTeam(5109);
    m_frontLeft = new SwerveModule((int) swerveFrontLeftMotors[0], (int) swerveFrontLeftMotors[1], (int) swerveFrontLeftMotors[2], swerveFrontLeftMotors[3]);
    m_frontRight = new SwerveModule((int) swerveFrontRightMotors[0], (int) swerveFrontRightMotors[1], (int) swerveFrontRightMotors[2], swerveFrontRightMotors[3]);
    m_backLeft = new SwerveModule((int) swerveBackLeftMotors[0], (int) swerveBackLeftMotors[1], (int) swerveBackLeftMotors[2], swerveBackLeftMotors[3]);
    m_backRight = new SwerveModule((int) swerveBackRightMotors[0], (int) swerveBackRightMotors[1], (int) swerveBackRightMotors[2], swerveBackRightMotors[3]);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    System.out.println(navX.isConnected());
    Rotation2d navXVal = new Rotation2d((-navX.getYaw() % 360 ) * Math.PI/180);
    SwerveModuleState[] swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, navXVal)
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);

    // for (SwerveModuleState state: swerveModuleStates) {
    //   System.out.println(state);
    // }
    // System.out.println("1");
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    // System.out.println("2");
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    // System.out.println("3");
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    // System.out.println("4");
    m_backRight.setDesiredState(swerveModuleStates[3]);
    // System.out.println(swerveModuleStates[0].speedMetersPerSecond);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        new Rotation2d(navX.getYaw() * 180/Math.PI),
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState());
  }

  /** Limelight autoalign method */
  public void autoAlign() throws Throwable {

    try {
      // Calls limelight method to get the z-distance from goal
      OptionalDouble straightDistanceOption = limelight.calculateYDistance();
      double straightDistance = straightDistanceOption.getAsDouble();

      // Offset variable initialization
      double yOffset = straightDistance - shooterRangeCm;
      double xOffset = straightDistance * Math.tan(limelight.getXOffset().getAsDouble() * Math.PI/180);

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
   * @return Returns the id of the listener
   */
  public int initBallListener() {
    NetworkTableEntry allianceEntry = ballAlignmentValues.getEntry("alliance");
    allianceEntry.setString(alliance.toString());
    int listenerHandle = ballAlignmentValues.addEntryListener("tVelocity", (table, key, entry, value, flags) -> {
      double[] velocity = value.getDoubleArray();
      double xVel = velocity[0];
      double yVel = velocity[1];

      if(xVel != 0 && yVel != 0){
        drive(xVel, yVel, 0, true);
      }
   }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
   return listenerHandle;
  }

  public int initShooterListener() {
    int listenerHandle = limelight.limelight.addEntryListener("tx",  (table, key, entry, value, flags) -> {
      try {
        autoAlign();
      } catch (NoSuchElementException e) {
        System.out.println("Target not found");
      }
      catch (Throwable e) {
        e.printStackTrace();
      }
    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    return listenerHandle;
  }
}
