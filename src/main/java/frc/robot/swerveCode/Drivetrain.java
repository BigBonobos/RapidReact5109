// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerveCode;


import frc.robot.Limelight;

import edu.wpi.first.wpilibj.SPI;

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
public class Drivetrain implements Runnable {

  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  // Network Table instantiation
  private final NetworkTableInstance ntwrkInst = NetworkTableInstance.getDefault();
  private NetworkTable ballAlignmentValues = ntwrkInst.getTable("ballAlignment");
  public volatile boolean runListener = false;
  public volatile boolean runAutoAlign = true;

  // Bot measurements
  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);
  private static final DriverStation.Alliance alliance = DriverStation.getAlliance();

  // Swerve Module instantiation
  private final SwerveModule m_frontLeft = new SwerveModule(1, 2);
  private final SwerveModule m_frontRight = new SwerveModule(3, 4);
  private final SwerveModule m_backLeft = new SwerveModule(5, 6);
  private final SwerveModule m_backRight = new SwerveModule(7, 8);
  private final AHRS navX = new AHRS(SPI.Port.kMXP);
  private final Rotation2d initialMeasurement = new Rotation2d((navX.getYaw() % 360) * Math.PI/180);

  // Shooter Range
  private double shooterRangeCm; // Enter shooter distance here(cm)
  private final Limelight limelight = new Limelight(61.49125);

  // Swerve drive library instantiation
  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(m_kinematics, initialMeasurement);


  /**
   * Constructor for the swerve drivetrain
   * @param shooterRange The optimal shooting distance in cm for the robot (used for auto aligment with limelight)
   */
  public Drivetrain(double shooterRange) {
    navX.reset();
    shooterRangeCm = shooterRange;
    ntwrkInst.startClientTeam(5109);
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
    Rotation2d navXVal = new Rotation2d((navX.getYaw() % 360 ) * Math.PI/180);
    SwerveModuleState[] swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, navXVal)
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
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
      double xOffset = limelight.getXOffset().getAsDouble();

      // Control logic to drive bot into position
      while (Math.abs(yOffset) > 2.5 && Math.abs(xOffset) > 2.5) {
          drive(xOffset / Math.PI, yOffset / Math.PI, 0.0, true);
          straightDistance = limelight.calculateYDistance().getAsDouble();
          xOffset = limelight.getXOffset().getAsDouble();
          yOffset = straightDistance - shooterRangeCm;
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
  public int initListener() {
    double rotation = (navX.getYaw() % 360) * 180/Math.PI;
    NetworkTableEntry rotEntry = ballAlignmentValues.getEntry("rot");
    NetworkTableEntry allianceEntry = ballAlignmentValues.getEntry("alliance");
    rotEntry.setDouble(rotation);
    allianceEntry.setString(alliance.toString());
    int listenerHandle = ballAlignmentValues.addEntryListener("tVelocity", (table, key, entry, value, flags) -> {
      if (runListener) {
        double[] velocity = value.getDoubleArray();
        double xVel = velocity[0];
        double yVel = velocity[1];
        drive(xVel, yVel, 0, true);
      }
   }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
   return listenerHandle;
  }

  @Override
  public void run() {
    try {
      while (runAutoAlign) {
        autoAlign();
        
        if (runAutoAlign) {
          runAutoAlign = false;
        }
      }
    } catch (NoSuchElementException e) {
      System.out.println("Vision target not detected (try pointing the robot towards the goal");
    } catch (Throwable e) {
      System.out.println("There is a likely an error with the limelight\nIt is reccomended you do not use auto-align for the rest of the match.\nPlease report this issue to programming.");
      e.printStackTrace();
    }
  };
}
