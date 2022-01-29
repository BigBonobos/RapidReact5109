// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.swerveCode.Drivetrain;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

public class Robot extends TimedRobot {

  // Ball Align Var
  private int listenerHandleBall;
  private int listenerHandleShooter;
  private boolean intakeRunning;

  // Variables for limelight alignment
  private boolean autoAlignRunningShooter = false;
  private boolean autoAlignRunningBall = false;
  private double autoAlignRange = 300.0;
  private int autoCounter = 0;

  // CAN IDs for swerve drivetrain
  private static final int[] frontLeftIds = {1, 2};
  private static final int[] frontRightIds = {3, 4};
  private static final int[] backLeftIds = {5, 6};
  private static final int[] backRightIds = {7, 8};

  private final Joystick l_joystick = new Joystick(0);
  private final Joystick r_joystick = new Joystick(1);
  private final Joystick j_operator = new Joystick(2);

  private final Drivetrain m_swerve = new Drivetrain(autoAlignRange, frontLeftIds, frontRightIds, backLeftIds, backRightIds);
  private final Intake m_intake = new Intake(9, 1, 2, 0);

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  @Override
  public void testPeriodic() {
    driveWithJoystick(true);
  }

  @Override
  public void robotInit() {
    listenerHandleBall = m_swerve.initBallListener();
  }

  @Override
  public void autonomousInit() {
    m_intake.ballIndexer = 1;
    m_intake.intakeOn = false;
  }
  @Override
  public void autonomousPeriodic() {
    // Sets TeleOp Mode to false
    driveWithJoystick(false);

    // Moves in the periodic loop from one instruction to another, useful for redundancy and testing
    switch(autoCounter) {
      case 1:
        // Sets intake power
        m_intake.intake(true);

        // While no limit switch input, wait until ball is aligned
        while(m_intake.ballIndexer == 1) {
          m_intake.checkIntakeState();
          Timer.delay(0.001);
        }

        // Remove ball aligner, and stop intaking
        m_swerve.ballAlignmentValues.removeEntryListener(listenerHandleBall);
        m_intake.intake(false);
        autoCounter++;
        break;
      
      case 2:
        // Shooter alignmnet
        try {
          listenerHandleShooter = m_swerve.initShooterListener();
        } catch (Throwable e) {
          e.printStackTrace();
        }
        break;
    }
    m_swerve.updateOdometry();
  }

  @Override
  public void teleopInit() {

    // Removes the shooter listener that was set in auto period
    m_swerve.limelight.limelight.removeEntryListener(listenerHandleShooter);

    // When intake is pressed the first time it will run
    intakeRunning = false;
  }
  
  @Override
  public void teleopPeriodic() {
    driveWithJoystick(true);

    // Comment the below code out for swerve testing

    // If the trigger is pressed, and autoAlign through limelight is not listening, then it will initialize the listener
    if (l_joystick.getTrigger() && !autoAlignRunningShooter && !autoAlignRunningBall) {
      autoAlignRunningShooter = true;
      listenerHandleShooter = m_swerve.initShooterListener();
    } 
    
    // Toggles above code off
    else if (l_joystick.getTrigger() && autoAlignRunningShooter && !autoAlignRunningBall || (Math.abs(m_swerve.limelight.limelight.getEntry("tx").getDouble(0.0)) < .5 && (Math.abs(m_swerve.limelight.limelight.getEntry("ty").getDouble(m_swerve.shooterRangeCm) - m_swerve.shooterRangeCm) < .5))) {
      m_swerve.limelight.limelight.removeEntryListener(listenerHandleShooter);
      autoAlignRunningShooter = false;
    }


    // If button 1 is pressed, trigger alignemnet code
    if (l_joystick.getRawButton(1) && !autoAlignRunningBall && m_intake.ballIndexer < 2 && !autoAlignRunningShooter) {
      listenerHandleBall = m_swerve.initBallListener();
      autoAlignRunningBall = true;
    } 
    
    // Toggle off for above code
    else if((l_joystick.getRawButton(1) && autoAlignRunningBall || m_intake.ballIndexer >= 2) && !autoAlignRunningShooter) {
      m_swerve.ballAlignmentValues.removeEntryListener(listenerHandleBall);
      autoAlignRunningBall = false;
    }

    // Intake toggle
    if(j_operator.getRawButton(1) && !intakeRunning) {
      m_intake.intake(!intakeRunning);
    } else if(j_operator.getRawButton(1) && intakeRunning){
      m_intake.intake(!intakeRunning);
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
