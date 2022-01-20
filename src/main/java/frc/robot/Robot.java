// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.swerveCode.Drivetrain;

enum DriveDirection {
  Angular,
  XDir,
  YDir
}

public class Robot extends TimedRobot {
  private final double shooterRangeCm = 5.0; // Enter shooter distance here(cm)
  private final Limelight limelight = new Limelight(61.49125);

  private final Joystick l_joystick = new Joystick(0);
  private final Joystick r_joystick = new Joystick(1);
  private final Drivetrain m_swerve = new Drivetrain();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  @Override
  public void autonomousPeriodic() {
    driveWithJoystick(false);
    m_swerve.updateOdometry();
  }

  @Override
  public void teleopPeriodic() {
    driveWithJoystick(true);
    if (l_joystick.getTrigger()) {
      autoAlign();
    }
  }

  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final double xSpeed =
        -m_xspeedLimiter.calculate(l_joystick.getY())
            * frc.robot.swerveCode.Drivetrain.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final double ySpeed =
        -m_yspeedLimiter.calculate(l_joystick.getX())
            * frc.robot.swerveCode.Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final double rot =
        -m_rotLimiter.calculate(r_joystick.getX())
            * frc.robot.swerveCode.Drivetrain.kMaxAngularSpeed;

    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
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
          m_swerve.drive(0, -Math.PI/2, 0.0, true);
          distanceInformation = limelight.calculate3dDistance();
          straightDistance = distanceInformation[0];
          offset = straightDistance - shooterRangeCm;
        }
        driveUntilAdjusted(DriveDirection.XDir);
      } else {
        while (Math.abs(offset) > 5.0) {
          m_swerve.drive(0, Math.PI/2, 0.0, true);
          distanceInformation = limelight.calculate3dDistance();
          angledDistance = distanceInformation[1];
          offset = angledDistance - shooterRangeCm;
        }
        driveUntilAdjusted(DriveDirection.Angular);
      }
    }
  }

  private void driveUntilAdjusted(DriveDirection direction) {
    double angle = limelight.calculateAngleOffset();
    double currentAngle = m_swerve.navX.getYaw();
    double startAngle = m_swerve.navX.getYaw();

    switch (direction) {
      case Angular:
        m_swerve.drive(0, 0, Integer.signum((int) angle) * Math.PI/2, true);
        break;
      case XDir:
        m_swerve.drive(Integer.signum((int) angle) * Math.PI/2, 0, 0.0, true);
        break;
      case YDir:
        m_swerve.drive(0, Integer.signum((int) angle) * Math.PI/2, 0.0, true);
        break;
    }
    while(Math.abs(angle - (currentAngle - startAngle)) > 2){
      angle = limelight.calculateAngleOffset();
    }
    m_swerve.drive(0, 0, 0, true);
  };
}
