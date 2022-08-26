// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.auto.Autonomous;
// import frc.robot.ballSys.Intake;
// import frc.robot.ballSys.Shooter;
// import frc.robot.ballSys.Shooter.ShooterState;
import frc.robot.swerveCode.Drivetrain;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

import java.util.Optional;

import javax.swing.text.StyledEditorKit;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Notifier;

import edu.wpi.first.math.MathUtil;

public class Robot extends TimedRobot {

  // public CANSparkMax motor1 = new CANSparkMax(14,MotorType.kBrushless);

  // Ball Align Var
  private int listenerHandleBall;
  private int listenerHandleShooter;
  private boolean intakeRunning;
  private boolean autoAlignRunningBall;
  private boolean autoAlignRunningShooter;

  /**
   * Variable assignment for limelight.
   * Currently unnused.
   */
  // private boolean autoAlignRunningShooter = false;
  // private boolean autoAlignRunningBall = false;
  private double autoAlignRange = 360.0;
  private BallSystems ballSys = new BallSystems();
  private Notifier ballSysNotif = new Notifier(ballSys);
  public Climb climb = new Climb();
  private int autoCounter = 1;

  /**
   * Settings for drivetrainModule.
   * 
   * @see {@link frc.robot.swerveCode.SwerveModule#SwerveModule(int, int, int, double)}
   */
  private static final double[] frontLeftIds = { 15, 14, 4, 162.6 };// 100}; // back right?
  private static final double[] frontRightIds = { 12, 13, 1, -163.213 };// (180 - 55) - 360}; // back left?
  private static final double[] backLeftIds = { 18, 19, 3, 60.6 };// -5}; //front right?
  private static final double[] backRightIds = { 16, 17, 2, -80.86 };// (180 + 40) - 360}; //front left?y

  private final XboxController xController = new XboxController(0);

  /**
   * Used to control other aspects of the bot (modules).
   */
  private final Joystick j_operator = new Joystick(1);

  /**
   * A custom implementation of four SwerveModules (also custom).
   * Can omni-directionally drive.
   * 
   * @see {@link frc.robot.Robot#driveWithJoystick(boolean)};
   */
  public final Drivetrain m_swerve = new Drivetrain(autoAlignRange, frontLeftIds, frontRightIds, backLeftIds,
      backRightIds);

  /**
   * Custom intake module. Takes one motor and one solenoid.
   * 
   * @see {@link frc.robot.shooter.Intake#Intake(int, int)}
   */
  // private final Intake m_intake = new Intake(9, 0);

  /**
   * SlewRateLimiters limit the ROC of an inputs strength.
   * <p>
   * For example, cannot instantly go from rest -> full speed.
   * </p>
   * <p>
   * Technical term is "interpolating" inbetween points until reaching full speed.
   * </p>
   * Basically, makes controls more gradual.
   * Lower = jerkier. Higher = smoother but slower.
   * <p>
   * setting of 10: 1/3 sec from 0 to 1.
   * </p>
   */
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(10);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(10);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(1);

  /**
   * Test stub. Called once upon initialization.
   */
  public void testInit() {
    m_swerve.customAutonAlign();
    ballSys.intakeOn = false;
    ballSys.BoolBall = false;
    ballSys.shooting = false;
  }

  /**
   * Test stub. This is a continual loop. 20ms pause between iterations.
   */
  @Override
  public void testPeriodic() {
    // System.out.println(m_swerve.m_frontLeft.m_turningEncoderAbsolute.getAbsolutePosition());
    // System.out.println(m_swerve.m_frontLeft.m_turningEncoderAbsolute.getAbsolutePosition());

    ballSys.m_indexWheel.set(1);
    ballSys.m_intakeWheel.set(1);
    ballSys.m_shooterWheel.set(0.5);



    SmartDashboard.putNumber("RPMS", ballSys.e_shooterWheel.getVelocity());
    SmartDashboard.putNumber("BallCount", ballSys.BallCount);
    ballSys.handleInputs(xController, j_operator);
    climb.handleInputs(xController, j_operator);
    // if(j_operator.getTrigger()) {
    // climb.popArmUp();
    // }
    // if (j_operator.getRawButton(3)) {
    // climb.popArmDown();
    // }
    // if (xController.getXButton()) {
    // switch (intakeState) {
    // case Stopped:
    // ballSys.m_intakeWheel.set(0.4);
    // intakeState = IntakeState.Go;
    // break;
    // case Go:
    // ballSys.m_intakeWheel.set(-0.4);
    // intakeState = IntakeState.Reverse;
    // case Reverse:
    // ballSys.m_intakeWheel.set(0);
    // intakeState = IntakeState.Stopped;
    // }
    // ballSys.m_intakeWheel.set(0.4);
    // }

    // if (xController.getYButton()) {
    // if(!ballSys.Beam2.get())
    // ballSys.m_indexWheel.set(0.4);
    // } else {
    // ballSys.m_indexWheel.set(0);
    // }

    // if(xController.getBButton()) {
    // ballSys.m_shooterWheel.set(0.9);
    // } else if (xController.getRightTriggerAxis() == 1) {
    // ballSys.m_shooterWheel.set(0.50);
    // } else {
    // ballSys.m_shooterWheel.set(0);
    // }
    // m_swerve.m_frontLeft.m_turningMotor.set(0.1);
    // Timer.delay(5);
    // m_swerve.m_frontLeft.m_turningMotor.set(0);
    // try {
    // Thread.sleep(100);
    // } catch (InterruptedException e) {
    // // TODO Auto-generated catch block
    // e.printStackTrace();
    // }
    // m_swerve.drive(0.1, 0.1, 0, false);

  }

  // @Override
  // public void robotInit() {
  // listenerHandleBall = m_swerve.initBallListener();
  // }

  @Override
  public void autonomousInit() {
    m_swerve.customAutonAlign();
    Timer.delay(1);
    m_swerve.navX.reset();
    autoCounter = 1;
    // ballSys.intakeOn = false;
    ballSys.BoolBall = false;
    ballSys.shooting = false;
    ballSys.BallCount = 1;

  }

  @Override
  public void autonomousPeriodic() {
    // System.out.println(ballSys.BallCount);
    // ballSys.Index();
    // // System.out.println(autoCounter);
    // System.out.println(Math.abs(15 - m_swerve.navX.getYaw()));
    // // Moves in the periodic loop from one instruction to another, useful for
    // redundancy and testing
    switch (autoCounter) {
      case -1:
        m_swerve.rotateToLimelightWanted();
        m_swerve.auto.stop();
        // if (Math.abs(15-m_swerve.navX.getYaw()) > .5) {

        // m_swerve.drive(0, 0, -.1, true);
        // } else {
        // m_swerve.drive(0, 0, 0, true);
        // autoCounter++;
        // }
        break;
      case -2:
        Timer.delay(1);
        ballSys.shooting2(true);
        autoCounter++;
        break;
      case 1:
        m_swerve.drive(.1, 0, 0, true);
        Timer.delay(5);
        m_swerve.drive(0, 0, 0, true);
        Timer.delay(3);
        // m_swerve.drive(0, .1, 0, true);
        autoCounter++;
        break;
    }
    // m_swerve.updateOdometry();
  }

  // // Remove ball aligner, and stop intaking
  // m_swerve.ballAlignmentValues.removeEntryListener(listenerHandleBall);
  // m_intake.intake(false);
  // autoCounter++;
  // break;

  // case 2:
  // // Shooter alignmnet
  // try {
  // listenerHandleShooter = m_swerve.initShooterListener();
  // autoCounter++;
  // } catch (Throwable e) {
  // e.printStackTrace();
  // }
  // break;

  // case 3:
  // m_shooter.state = ShooterState.kRunning;
  // autoCounter++;
  // break;

  // case 4:
  // Timer.delay(5);
  // // Put Ball deploy method here
  // autoCounter++;
  // break;

  // case 5:
  // m_shooter.state = ShooterState.kCoasting;
  // break;
  // }
  // m_swerve.updateOdometry();
  // }

  /**
   * Init when setting up teleop setting.
   */
  @Override
  public void teleopInit() {
    m_swerve.customTeleopAlign();
  }

  /**
   * Init when setting up teleop setting.
   */
  @Override
  public void teleopPeriodic() {
    driveWithJoystick(true);
    // m_shooter.setShooter();
    // // Comment the below code out for swerve testing

    // // If the trigger is pressed, and autoAlign through limelight is not
    // listening, then it will initialize the listener
    // if (l_joystick.getTrigger() && !autoAlignRunningShooter &&
    // !autoAlignRunningBall) {
    // autoAlignRunningShooter = true;
    // listenerHandleShooter = m_swerve.initShooterListener();
    // }

    // Toggles above code off
    // else if (l_joystick.getTrigger() && autoAlignRunningShooter &&
    // !autoAlignRunningBall ||
    // (Math.abs(m_swerve.limelight.limelight.getEntry("tx").getDouble(0.0)) < .5 &&
    // (Math.abs(m_swerve.limelight.limelight.getEntry("ty").getDouble(m_swerve.shooterRangeCm)
    // - m_swerve.shooterRangeCm) < .5))) {
    // m_swerve.limelight.limelight.removeEntryListener(listenerHandleShooter);
    // autoAlignRunningShooter = false;
    // }

    // If button 1 is pressed, trigger alignemnet code
    // if (l_joystick.getRawButton(1) && !autoAlignRunningBall &&
    // m_intake.ballIndexer < 2 && !autoAlignRunningShooter) {
    // listenerHandleBall = m_swerve.initBallListener();
    // autoAlignRunningBall = true;
    // }

    // Toggle off for above code
    // else if((l_joystick.getRawButton(1) && autoAlignRunningBall ||
    // m_intake.ballIndexer >= 2) && !autoAlignRunningShooter) {
    // m_swerve.ballAlignmentValues.removeEntryListener(listenerHandleBall);
    // autoAlignRunningBall = false;
    // }

    // Intake toggle
    // if(j_operator.getRawButton(1) && !intakeRunning) {
    // m_intake.intake(!intakeRunning);
    // } else if(j_operator.getRawButton(1) && intakeRunning){
    // m_intake.intake(!intakeRunning);
    // }

    // if (j_operator.getTrigger()) {
    // switch (m_shooter.state) {
    // case kCoasting:
    // m_shooter.state = ShooterState.kRunning;
    // break;
    // case kRunning:
    // m_shooter.state = ShooterState.kCoasting;
    // break;
    // }
    // }
  }

  private void driveWithJoystick(boolean fieldRelative) {
    /**
     * Get desired X speed of chassis.
     * Inverted since Xbox joysticks return flipped values.
     * 
     **/
    final double xSpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband(xController.getLeftY(), 0.08))
        * frc.robot.swerveCode.Drivetrain.kMaxSpeed;
    // System.out.println(xSpeed);

    /**
     * Get desired Y (strafe/sideways) speed of chassis.
     * Positive = left, negative = right.
     * XBox controllers return flipped values.
     **/
    final double ySpeed = -m_yspeedLimiter.calculate(MathUtil.applyDeadband(xController.getLeftX(), 0.08))
        * frc.robot.swerveCode.Drivetrain.kMaxSpeed;
    // System.out.println(ySpeed);

    /**
     * Get desired rotation speed of chassis.
     * Positive = left, negative = right.
     * Xbox returns positive when holding right by default.
     **/
    final double rot = -m_rotLimiter.calculate(MathUtil.applyDeadband(xController.getRightX(), 0.12))
        * frc.robot.swerveCode.Drivetrain.kMaxAngularSpeed;
    // System.out.println(rot);
    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
  }
}
