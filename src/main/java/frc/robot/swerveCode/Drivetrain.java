// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerveCode;

import edu.wpi.first.wpilibj.SPI;

import com.kauailabs.navx.frc.*;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Limelight;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain implements Runnable{

  /** Enum to represent three directions of movement possible with swerve drive. */
  private enum DriveDirection {
    Angular,
    XDir,
    YDir
  }

  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  
  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

  private final SwerveModule m_frontLeft = new SwerveModule(1, 2);
  private final SwerveModule m_frontRight = new SwerveModule(3, 4);
  private final SwerveModule m_backLeft = new SwerveModule(5, 6);
  private final SwerveModule m_backRight = new SwerveModule(7, 8);
  private final AHRS navX = new AHRS(SPI.Port.kMXP);
  private final Rotation2d initialMeasurement = new Rotation2d((navX.getYaw() % 360) * Math.PI/180);

  private final double shooterRangeCm = 5.0; // Enter shooter distance here(cm)
  private final Limelight limelight = new Limelight(61.49125);

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(m_kinematics, initialMeasurement);

  public Drivetrain() {
    navX.reset();
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

  private void autoAlign() {
    double[] distanceInformation = limelight.calculate3dDistance();
    double straightDistance = distanceInformation[0];
    double angledDistance = distanceInformation[1];
    if (Math.abs(shooterRangeCm - straightDistance) <= 5.0) {
      driveUntilAdjusted(DriveDirection.XDir);
    }
    else if (Math.abs(shooterRangeCm - angledDistance) <= 5.0) {
      driveUntilAdjusted(DriveDirection.Angular);
    } else {
      double offset = straightDistance - shooterRangeCm;
      if (offset < 0) {
        while (Math.abs(offset) > 5.0) {
          drive(0, -Math.PI/2, 0.0, true);
          distanceInformation = limelight.calculate3dDistance();
          angledDistance = distanceInformation[1];
          offset = angledDistance - shooterRangeCm;
        }
        driveUntilAdjusted(DriveDirection.XDir);
      } else {
        while (Math.abs(offset) > 5.0) {
          drive(0, Math.PI/2, 0.0, true);
          distanceInformation = limelight.calculate3dDistance();
          straightDistance = distanceInformation[0];
          offset = straightDistance - shooterRangeCm;
        }
        driveUntilAdjusted(DriveDirection.Angular);
      }
    }
  }

  private void driveUntilAdjusted(DriveDirection direction) {
    double angle = limelight.calculateAngleOffset();
    double currentAngle = navX.getYaw();
    double startAngle = navX.getYaw();

    switch (direction) {
      case Angular:
        drive(0, 0, Integer.signum((int) angle) * Math.PI/2, true);
        break;
      case XDir:
        drive(Integer.signum((int) angle) * Math.PI/2, 0, 0.0, true);
        break;
      case YDir:
        drive(0, Integer.signum((int) angle) * Math.PI/2, 0.0, true);
        break;
    }
    while(Math.abs(angle - (currentAngle - startAngle)) > 2){
      angle = limelight.calculateAngleOffset();
    }
    drive(0, 0, 0, true);
  }

  @Override
  public void run() {
    autoAlign();
  };
}
