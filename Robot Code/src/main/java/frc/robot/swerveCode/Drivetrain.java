// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerveCode;

import frc.robot.Limelight;
import frc.robot.autonomous.Autonomous;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;

import java.util.NoSuchElementException;
import java.util.Optional;
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

  public static final double kMaxSpeed = 0.3; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI / 6; // 1/2 rotation per second
  public double globalX = 0;
  public double globalY = 0;

  // Network Table instantiation
  private final NetworkTableInstance ntwrkInst = NetworkTableInstance.getDefault();
  public NetworkTable ballAlignmentValues = ntwrkInst.getTable("ballAlignment");

  // Bot measurements
  private final Translation2d m_frontLeftLocation = new Translation2d(0.2921, 0.2921);
  private final Translation2d m_frontRightLocation = new Translation2d(0.2921, -0.2921);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.2921, 0.2921);
  private final Translation2d m_backRightLocation = new Translation2d(-0.2921, -0.2921);
  private static final DriverStation.Alliance alliance = DriverStation.getAlliance();

  // Swerve Module instantiation
  public final SwerveModule m_frontLeft;
  public final SwerveModule m_frontRight;
  public final SwerveModule m_backLeft;
  public final  SwerveModule m_backRight;
  public final AHRS navX = new AHRS(SPI.Port.kMXP);
  private final Rotation2d initialMeasurement = Rotation2d.fromDegrees(navX.getYaw() % 360);

  // Shooter Range
  public double shooterRangeCm; // Enter shooter distance here (cm)
  public final Limelight limelight = new Limelight(6*2.51);

  // Swerve drive library instantiation
  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  public final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, initialMeasurement);

  public final Autonomous auto = new Autonomous(this);

  /**
   * Constructor for the swerve drivetrain
   * 
   * @param shooterRange           The optimal shooting distance in cm for the
   *                               robot (used for auto aligment with limelight)
   * @param swerveFrontLeftMotors  The CAN ids for the swerve modules. The first
   *                               element in the array should be the drive motor
   *                               id, the second should be the turning motor id
   * @param swerveFrontRightMotors The CAN ids for the swerve modules. The first
   *                               element in the array should be the drive motor
   *                               id, the second should be the turning motor id
   * @param swerveBackLeftMotors   The CAN ids for the swerve modules. The first
   *                               element in the array should be the drive motor
   *                               id, the second should be the turning motor id
   * @param swerveBackRightMotors  The CAN ids for the swerve modules. The first
   *                               element in the array should be the drive motor
   *                               id, the second should be the turning motor id
   */
  public Drivetrain(double shooterRange, double[] swerveFrontLeftMotors, double[] swerveFrontRightMotors,
      double[] swerveBackLeftMotors, double[] swerveBackRightMotors) {
    navX.reset();
    shooterRangeCm = shooterRange;
    ntwrkInst.startClientTeam(5109);
    m_frontLeft = new SwerveModule((int) swerveFrontLeftMotors[0], (int) swerveFrontLeftMotors[1],
        (int) swerveFrontLeftMotors[2], swerveFrontLeftMotors[3]);
    m_frontRight = new SwerveModule((int) swerveFrontRightMotors[0], (int) swerveFrontRightMotors[1],
        (int) swerveFrontRightMotors[2], swerveFrontRightMotors[3]);
    m_backLeft = new SwerveModule((int) swerveBackLeftMotors[0], (int) swerveBackLeftMotors[1],
        (int) swerveBackLeftMotors[2], swerveBackLeftMotors[3]);
    m_backRight = new SwerveModule((int) swerveBackRightMotors[0], (int) swerveBackRightMotors[1],
        (int) swerveBackRightMotors[2], swerveBackRightMotors[3]);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    Rotation2d navXVal = new Rotation2d((-navX.getYaw() % 360) * Math.PI / 180);
    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, navXVal)
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);

    // for (SwerveModuleState state: swerveModuleStates) {
    // System.out.println(state);
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
        new Rotation2d(navX.getYaw() * 180 / Math.PI),
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState());
  }

  /** Limelight autoalign method */
  public void autoAlign() {
    OptionalDouble xOffset = limelight.getXOffset();
    if (xOffset.isPresent()) {
      while(xOffset.getAsDouble() != 0) {
        drive(0, 0, xOffset.getAsDouble(), false);
      }
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
    double degreeOffset = 10;

    double e_frontLeftPos = m_frontLeft.m_turningEncoderAbsolute.getAbsolutePosition();
    double e_frontRightPos = m_frontRight.m_turningEncoderAbsolute.getAbsolutePosition();
    double e_backRightPos = m_backRight.m_turningEncoderAbsolute.getAbsolutePosition();
    double e_backLeftPos = m_backLeft.m_turningEncoderAbsolute.getAbsolutePosition();
    // boolean outuput =Math.abs(e_backLeftPos) <= degreeOffset &&
    // Math.abs(e_frontLeftPos) <= degreeOffset && Math.abs(e_frontRightPos) <=
    // degreeOffset && Math.abs(e_backRightPos) <= degreeOffset;

    while (testCounter == 0) {
      if (Math.abs(e_backLeftPos) <= degreeOffset && Math.abs(e_frontLeftPos) <= degreeOffset
          && Math.abs(e_frontRightPos) <= degreeOffset && Math.abs(e_backRightPos) <= degreeOffset) {
        testCounter = 1;
      }
      e_frontLeftPos = m_frontLeft.m_turningEncoderAbsolute.getAbsolutePosition();
      e_frontRightPos = m_frontRight.m_turningEncoderAbsolute.getAbsolutePosition();
      e_backRightPos = m_backRight.m_turningEncoderAbsolute.getAbsolutePosition();
      e_backLeftPos = m_backLeft.m_turningEncoderAbsolute.getAbsolutePosition();

      // boolean output = Math.abs(e_backLeftPos) <= degreeOffset &&
      // Math.abs(e_frontLeftPos) <= degreeOffset && Math.abs(e_frontRightPos) <=
      // degreeOffset && Math.abs(e_backRightPos) <= degreeOffset;

      // System.out.println(-1 * e_frontLeftPos/180);
      // System.out.println(-1 * e_frontRightPos/180);
      // System.out.println(-1 * e_backRightPos/180);
      // System.out.println(-1 * e_frontLeftPos/180);

      m_frontLeft.m_turningMotor.set(-1 * e_frontLeftPos / 180);
      m_backLeft.m_turningMotor.set(-1 * e_backLeftPos / 180);
      m_frontRight.m_turningMotor.set(-1 * e_frontRightPos / 180);
      m_backRight.m_turningMotor.set(-1 * e_backRightPos / 180);
    }
  }

  public void autoAlignToGoalUsingLimelight() {
    OptionalDouble posRes = limelight.getXOffset();
    if (posRes.isPresent()) {
      double res = posRes.getAsDouble();
      auto.rotateTo(Rotation2d.fromDegrees(navX.getYaw() + res));
    } else {
      System.out.println("No object detected.");
    }

  }
}
