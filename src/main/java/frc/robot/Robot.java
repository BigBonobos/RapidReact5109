// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.swerveCode.Drivetrain;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

public class Robot extends TimedRobot {

  // Variables for limelight alignment
  private boolean autoAlignRunningShooter = false;
  private boolean autoAlignRunningBall = false;
  private double autoAlignRange = 300.0;
  private int autoCounter = 0;

  private final Joystick l_joystick = new Joystick(0);
  private final Joystick r_joystick = new Joystick(1);
  private final Drivetrain m_swerve = new Drivetrain(autoAlignRange);
  private final Intake m_intake = new Intake(9, 1, 2, 0);

  private final Notifier autoAlignNotif = new Notifier(m_swerve);
  private final Notifier intakeNotif = new Notifier(m_intake);

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  @Override
  public void testPeriodic() {
    driveWithJoystick(true);
  }

  /*
  Commented out because not relevant to testing
  @Override
  public void robotInit() {
    m_swerve.initListener();
  }
  */

  @Override
  public void autonomousInit() {
    m_intake.ballIndexer = 1;
    m_intake.intakeOn = false;
  }
  @Override
  public void autonomousPeriodic() {
    driveWithJoystick(false);
    switch(autoCounter) {
      case 1:
        m_swerve.runListener = true;
        intakeNotif.startSingle(0);
        while(m_intake.ballIndexer == 1) {
          Timer.delay(0.001);
        }
        intakeNotif.stop();
        autoCounter++;
        break;
      case 2:
        m_swerve.runListener = false;
        try {
          m_swerve.autoAlign();
        } catch (Throwable e) {
          e.printStackTrace();
        }
        break;
    }
    m_swerve.updateOdometry();
  }

  @Override
  public void teleopInit() {
    m_swerve.runListener = false;
  }
  @Override
  public void teleopPeriodic() {
    driveWithJoystick(true);

    // Comment the below code out for swerve testing
    if (l_joystick.getTrigger() && !autoAlignRunningShooter) {
      autoAlignRunningShooter = true;
      autoAlignNotif.startSingle(0);
    } else if (l_joystick.getTrigger() && autoAlignRunningShooter) {
      autoAlignNotif.stop();
      autoAlignRunningShooter = false;
    }
    if (l_joystick.getRawButton(1) && !autoAlignRunningBall) {
      m_swerve.runListener = true;
      autoAlignRunningBall = true;
    } else if(l_joystick.getRawButton(0) && autoAlignRunningBall) {
      m_swerve.runListener = false;
      autoAlignRunningBall = false;
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
}
