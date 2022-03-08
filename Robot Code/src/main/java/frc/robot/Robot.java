// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


// import frc.robot.ballSys.Intake;
// import frc.robot.ballSys.Shooter;
// import frc.robot.ballSys.Shooter.ShooterState;
import frc.robot.swerveCode.Drivetrain;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Notifier;

import edu.wpi.first.math.MathUtil;

public class Robot extends TimedRobot {

  enum IntakeState {
    Stopped,
    Go,
    Reverse
  }

  // public CANSparkMax motor1 = new CANSparkMax(14,MotorType.kBrushless);

  // Ball Align Var
  private int listenerHandleBall;
  private int listenerHandleShooter;
  private boolean intakeRunning;
  private boolean autoAlignRunningBall;
  private boolean autoAlignRunningShooter;

  // Variables for limelight alignment
  // private boolean autoAlignRunningShooter = false;
  // private boolean autoAlignRunningBall = false;
  private double autoAlignRange = 360.0;
  private BallSystems ballSys = new BallSystems();
  private Notifier ballSysNotif = new Notifier(ballSys);
  public IntakeState intakeState = IntakeState.Stopped;
  public Climb climb = new Climb();
  private int autoCounter = 1;

  // CAN IDs for swerve drivetrain
  private static final double[] frontLeftIds = {15, 14, 4, 162.6};//100}; // back right? 
  private static final double[] frontRightIds = {12, 13, 1, -163.213};//(180 - 55) - 360}; // back left?
  private static final double[] backLeftIds = {18, 19, 3, 60.6};//-5}; //front right?
  private static final double[] backRightIds = {16, 17, 2, -80.86};//(180 + 40) - 360}; //front left?y

  private final XboxController xController = new XboxController(0);
  // private final Joystick r_joystick = new Joystick(1);
  private final Joystick j_operator = new Joystick(1);

  private final Drivetrain m_swerve = new Drivetrain(autoAlignRange, frontLeftIds, frontRightIds, backLeftIds, backRightIds);
  // private final Intake m_intake = new Intake(9, 0);

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(10);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(10);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(1);

  double testCounter = 0;
  double degreeOffset = 10;
  private void initSwerve() {
    double e_frontLeftPos = m_swerve.m_frontLeft.m_turningEncoderAbsolute.getAbsolutePosition();
    double e_frontRightPos = m_swerve.m_frontRight.m_turningEncoderAbsolute.getAbsolutePosition();
    double e_backRightPos = m_swerve.m_backRight.m_turningEncoderAbsolute.getAbsolutePosition();
    double e_backLeftPos = m_swerve.m_backLeft.m_turningEncoderAbsolute.getAbsolutePosition();
    // boolean outuput =Math.abs(e_backLeftPos) <= degreeOffset && Math.abs(e_frontLeftPos) <= degreeOffset && Math.abs(e_frontRightPos) <= degreeOffset && Math.abs(e_backRightPos) <= degreeOffset;

    while(testCounter == 0) {
      if (Math.abs(e_backLeftPos) <= degreeOffset && Math.abs(e_frontLeftPos) <= degreeOffset && Math.abs(e_frontRightPos) <= degreeOffset && Math.abs(e_backRightPos) <= degreeOffset) {
        testCounter = 1;
      }
      e_frontLeftPos = m_swerve.m_frontLeft.m_turningEncoderAbsolute.getAbsolutePosition();
      e_frontRightPos = m_swerve.m_frontRight.m_turningEncoderAbsolute.getAbsolutePosition();
      e_backRightPos = m_swerve.m_backRight.m_turningEncoderAbsolute.getAbsolutePosition();
      e_backLeftPos = m_swerve.m_backLeft.m_turningEncoderAbsolute.getAbsolutePosition(); 

      // boolean output = Math.abs(e_backLeftPos) <= degreeOffset && Math.abs(e_frontLeftPos) <= degreeOffset && Math.abs(e_frontRightPos) <= degreeOffset && Math.abs(e_backRightPos) <= degreeOffset;

      // System.out.println(-1 * e_frontLeftPos/180);
      // System.out.println(-1 * e_frontRightPos/180);
      // System.out.println(-1 * e_backRightPos/180);
      // System.out.println(-1 * e_frontLeftPos/180);


      m_swerve.m_frontLeft.m_turningMotor.set(-1 * e_frontLeftPos/180);
      m_swerve.m_backLeft.m_turningMotor.set(-1 * e_backLeftPos/180);
      m_swerve.m_frontRight.m_turningMotor.set(-1 * e_frontRightPos/180);
      m_swerve.m_backRight.m_turningMotor.set(-1 * e_backRightPos/180);
    }
  }
  public void testInit() {
    initSwerve();
    ballSys.intakeOn = false; 
    ballSys.BoolBall = false; 
    ballSys.shooting = false; 
  }
  

  @Override
  public void testPeriodic() {
    // System.out.println(m_swerve.m_frontLeft.m_turningEncoderAbsolute.getAbsolutePosition());
    // System.out.println(m_swerve.m_frontLeft.m_turningEncoderAbsolute.getAbsolutePosition());
    ballSys.Index();
    ballSys.intakeMotor();
    
    driveWithJoystick(true);

    if (xController.getRightTriggerAxis() == 1){
      ballSysNotif.startSingle(.001);
    } else if (xController.getRightBumper()) {
      ballSysNotif.stop();
    }
    // if(j_operator.getTrigger()) {
    //   climb.popArmUp();
    // }
    // if (j_operator.getRawButton(3)) {
    //   climb.popArmDown();
    // }
    // if (xController.getXButton()) {
    //   switch (intakeState) {
    //     case Stopped:
    //       ballSys.m_intakeWheel.set(0.4);
    //       intakeState = IntakeState.Go;
    //       break;
    //     case Go:
    //       ballSys.m_intakeWheel.set(-0.4);
    //       intakeState = IntakeState.Reverse;
    //     case Reverse:
    //       ballSys.m_intakeWheel.set(0);
    //       intakeState = IntakeState.Stopped;
    //   }
    //   ballSys.m_intakeWheel.set(0.4);
    // }

    // if (xController.getYButton()) {
    //   if(!ballSys.Beam2.get())
    //   ballSys.m_indexWheel.set(0.4);
    // } else {
    //   ballSys.m_indexWheel.set(0);
    // }

    // if(xController.getBButton()) {
    //   ballSys.m_shooterWheel.set(0.9);
    // }  else if (xController.getRightTriggerAxis() == 1) {
    //   ballSys.m_shooterWheel.set(0.50);
    // } else {
    //   ballSys.m_shooterWheel.set(0);
    // }
    // m_swerve.m_frontLeft.m_turningMotor.set(0.1);
    // Timer.delay(5);
    // m_swerve.m_frontLeft.m_turningMotor.set(0);
    // try {
    //   Thread.sleep(100);
    // } catch (InterruptedException e) {
    //   // TODO Auto-generated catch block
    //   e.printStackTrace();
    // }
    // m_swerve.drive(0.1, 0.1, 0, false);
 
  }

  // @Override
  // public void robotInit() {
  //   listenerHandleBall = m_swerve.initBallListener();
  // }

  @Override
  public void autonomousInit() {
    initSwerve();
  }
  @Override
  public void autonomousPeriodic() {

    // Moves in the periodic loop from one instruction to another, useful for redundancy and testing
    switch(autoCounter) {
      case 1:
        m_swerve.drive(0, -1, 0, true);
        Timer.delay(0.001);
        m_swerve.drive(0,0,0,true);
        break;
    }
    // m_swerve.updateOdometry();
  }

  @Override
  public void teleopInit() {
    double e_frontLeftPos = m_swerve.m_frontLeft.m_turningEncoderAbsolute.getAbsolutePosition();
    double e_frontRightPos = m_swerve.m_frontRight.m_turningEncoderAbsolute.getAbsolutePosition();
    double e_backRightPos = m_swerve.m_backRight.m_turningEncoderAbsolute.getAbsolutePosition();
    double e_backLeftPos = m_swerve.m_backLeft.m_turningEncoderAbsolute.getAbsolutePosition();
    // boolean outuput =Math.abs(e_backLeftPos) <= degreeOffset && Math.abs(e_frontLeftPos) <= degreeOffset && Math.abs(e_frontRightPos) <= degreeOffset && Math.abs(e_backRightPos) <= degreeOffset;

    while(testCounter == 0) {
      if (Math.abs(e_backLeftPos) <= degreeOffset && Math.abs(e_frontLeftPos) <= degreeOffset && Math.abs(e_frontRightPos) <= degreeOffset && Math.abs(e_backRightPos) <= degreeOffset) {
        testCounter = 1;
      }
      e_frontLeftPos = m_swerve.m_frontLeft.m_turningEncoderAbsolute.getAbsolutePosition();
      e_frontRightPos = m_swerve.m_frontRight.m_turningEncoderAbsolute.getAbsolutePosition();
      e_backRightPos = m_swerve.m_backRight.m_turningEncoderAbsolute.getAbsolutePosition();
      e_backLeftPos = m_swerve.m_backLeft.m_turningEncoderAbsolute.getAbsolutePosition(); 

      // boolean output = Math.abs(e_backLeftPos) <= degreeOffset && Math.abs(e_frontLeftPos) <= degreeOffset && Math.abs(e_frontRightPos) <= degreeOffset && Math.abs(e_backRightPos) <= degreeOffset;

      // System.out.println(-1 * e_frontLeftPos/180);
      // System.out.println(-1 * e_frontRightPos/180);
      // System.out.println(-1 * e_backRightPos/180);
      // System.out.println(-1 * e_frontLeftPos/180);


      m_swerve.m_frontLeft.m_turningMotor.set(-1 * e_frontLeftPos/180);
      m_swerve.m_backLeft.m_turningMotor.set(-1 * e_backLeftPos/180);
      m_swerve.m_frontRight.m_turningMotor.set(-1 * e_frontRightPos/180);
      m_swerve.m_backRight.m_turningMotor.set(-1 * e_backRightPos/180);
    }
  }
  
  @Override
  public void teleopPeriodic() {
    driveWithJoystick(true);
    // m_shooter.setShooter();
    // // Comment the below code out for swerve testing

    // // If the trigger is pressed, and autoAlign through limelight is not listening, then it will initialize the listener
    // if (l_joystick.getTrigger() && !autoAlignRunningShooter && !autoAlignRunningBall) {
    //   autoAlignRunningShooter = true;
    //   listenerHandleShooter = m_swerve.initShooterListener();
    // } 
    
    // Toggles above code off
    // else if (l_joystick.getTrigger() && autoAlignRunningShooter && !autoAlignRunningBall || (Math.abs(m_swerve.limelight.limelight.getEntry("tx").getDouble(0.0)) < .5 && (Math.abs(m_swerve.limelight.limelight.getEntry("ty").getDouble(m_swerve.shooterRangeCm) - m_swerve.shooterRangeCm) < .5))) {
    //   m_swerve.limelight.limelight.removeEntryListener(listenerHandleShooter);
    //   autoAlignRunningShooter = false;
    // }


    // If button 1 is pressed, trigger alignemnet code
    // if (l_joystick.getRawButton(1) && !autoAlignRunningBall && m_intake.ballIndexer < 2 && !autoAlignRunningShooter) {
    //   listenerHandleBall = m_swerve.initBallListener();
    //   autoAlignRunningBall = true;
    // } 
    
    // Toggle off for above code
    // else if((l_joystick.getRawButton(1) && autoAlignRunningBall || m_intake.ballIndexer >= 2) && !autoAlignRunningShooter) {
    //   m_swerve.ballAlignmentValues.removeEntryListener(listenerHandleBall);
    //   autoAlignRunningBall = false;
    // }

    // Intake toggle
    // if(j_operator.getRawButton(1) && !intakeRunning) {
    //   m_intake.intake(!intakeRunning);
    // } else if(j_operator.getRawButton(1) && intakeRunning){
    //   m_intake.intake(!intakeRunning);
    // }

  //   if (j_operator.getTrigger()) {
  //     switch (m_shooter.state) {
  //       case kCoasting:
  //         m_shooter.state = ShooterState.kRunning;
  //         break;
  //       case kRunning:
  //         m_shooter.state = ShooterState.kCoasting;
  //         break;
  //     }
  //   }
  }

  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final double xSpeed =
        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(xController.getLeftY(), 0.08))
            * frc.robot.swerveCode.Drivetrain.kMaxSpeed;
    // System.out.println(xSpeed);

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final double ySpeed =
        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(xController.getLeftX(), 0.08))
            * frc.robot.swerveCode.Drivetrain.kMaxSpeed;
    // System.out.println(ySpeed);

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final double rot =
        -m_rotLimiter.calculate(MathUtil.applyDeadband(xController.getRightX(), 0.12))
            * frc.robot.swerveCode.Drivetrain.kMaxAngularSpeed;
      // System.out.println(rot);
    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
  }
}
