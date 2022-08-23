// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.autonomous.Autonomous;
// import frc.robot.ballSys.AutoShoot;
import frc.robot.ballSys.BallFondler;
import frc.robot.climb.ClimbModule;
import frc.robot.swerveCode.Drivetrain;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

import java.util.Optional;
import java.util.OptionalDouble;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;

public class Robot extends TimedRobot {

  /**
   * Variable assignment for limelight.
   * Currently unnused.
   */
  private double autoAlignRange = 360.0;

  /**
   * Settings for our drive train.
   * 
   * @see {@link frc.robot.swerveCode.SwerveModule#SwerveModule(int, int, int, double)}
   */
  private static final double[] frontLeftIds = { 15, 14, 4, 162.6 };// 100}; // back right?
  private static final double[] frontRightIds = { 12, 13, 1, -163.213 };// (180 - 55) - 360}; // back left?
  private static final double[] backLeftIds = { 18, 19, 3, 60.6 };// -5}; //front right?
  private static final double[] backRightIds = { 16, 17, 2, -80.86 };// (180 + 40) - 360}; //front left?y

  /**
   * XboxController for general movement of the robot.
   */
  private final XboxController xController = new XboxController(0);

  /**
   * Used to control other aspects of the bot (modules).
   */
  private final Joystick j_operator = new Joystick(1);

  /**
   * A custom implementation of four SwerveModules (also custom).
   * Can omni-directionally drive.
   * 
   * @see {@link frc.robot.Robot#driveWithJoystick(boolean) Drivetrain
   *      constructor.}
   */
  public final Drivetrain m_swerve = new Drivetrain(Optional.of(autoAlignRange), frontLeftIds, frontRightIds, backLeftIds,
      backRightIds);

  /**
   * SlewRateLimiters limit the ROC of an inputs strength.
   * <p>
   * For example, cannot instantly go from rest -> full speed.
   * <p>
   * Technical term is "interpolating" inbetween points until reaching full speed.
   * 
   * Basically, makes controls more gradual.
   * Lower = jerkier. Higher = smoother but slower.
   * <p>
   * setting of 10: 1/3 sec from 0 to 1.
   */
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(10);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(10);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(1);

  /**
   * CANSparkMax motor ports.
   * <p>
   * {@link com.revrobotics.CANSparkMax#CANSparkMax(int, MotorType) motor config.}
   */
  private final int[] bfIDs = new int[] { 4, 22, 8 };

  /**
   * Handler for intake and shooter. Uses {@link Robot#bfIDs motor ids}
   * to set up.
   * 
   * @see {@link frc.robot.ballSys.BallFondler#BallFondler(int, int, int, double)
   *      BallFondler constructor.}
   */
  public final BallFondler ballFondler = new BallFondler(bfIDs[0], bfIDs[1], bfIDs[2], 3650);

  /**
   * Setup for solenoid ports for the climb module.
   * 
   * @see {@link frc.robot.climb.ClimbModule#ClimbModule(int, int[][]) ClimbModule
   *      constructor.}
   */
  private final int[][] solenoidPorts = new int[][] { { 9, 8 }, { 7, 6 } };

  /**
   * Climb module. Controls two solenoids and a motor.
   * 
   * @see {@link frc.robot.climb.ClimbModule#ClimbModule(int, int[][]) ClimbModule
   *      constructor.}
   */
  private final ClimbModule climbModule = new ClimbModule(10, solenoidPorts);

  /**
   * Index for autonomous.
   */
  private int autoCounter = 1;

  // public final AutoShoot autoShoot = new AutoShoot(this, new Translation2d(1, 0), 3, 10);


  /**
   * Ran once on bot initialization.
   */
  @Override
  public void robotInit() {
    // listenerHandleBall = m_swerve.initBallListener();
  }

  /**
   * Test stub. Called once upon initialization.
   */
  @Override
  public void testInit() {
    // m_swerve.navX.resetDisplacement();
    // System.out.println(new Translation2d(m_swerve.navX.getDisplacementX(), m_swerve.navX.getDisplacementY()));
    // m_swerve.customAutoAlign();
    // // m_swerve.auto.rotateTo(Rotation2d.fromDegrees(45));
    // // m_swerve.auto.translateTo(new Translation2d(.3, 0), 0.05);
    // System.out.println(autoShoot.shootAtGoal());
    // m_swerve.auto.stop();
    m_swerve.customAutoAlign();
    Timer.delay(1);
    // m_swerve.autoAlign();
  }

  /**
   * Test stub. This is a continual loop. 20ms pause between iterations.
   */
  @Override
  public void testPeriodic() {
    driveWithJoystick(true);
    // System.out.printf("%s\n",
    //     new Translation2d(m_swerve.navX.getDisplacementX(), m_swerve.navX.getDisplacementY()).toString());O

    handleInputs(xController, j_operator);

  }

  /**
   * Called once when entering autonomous mode.
   */
  @Override
  public void autonomousInit() {
    autoCounter = 1;
    ballFondler.hardReset();
    m_swerve.customAutoAlign();
    Timer.delay(1);
  }

  /**
   * Called every 20 ms during autonomous mode.
   */
  @Override
  public void autonomousPeriodic() {
    switch (autoCounter) {
      case 1:
        m_swerve.auto.rotateTo(Rotation2d.fromDegrees(45), 0.1);
        autoCounter++;
        break;
      case 2:
        if (ballFondler.getAndUpdateBallCount() >= 2) {
          ballFondler.intake.resetSystem();
          autoCounter++;
          break;
        }
        ballFondler.intake.intakeFor(1);
        autoCounter++;
        break;
      case 3:
        ballFondler.shoot();
        autoCounter++;
        break;

    }

    m_swerve.updateOdometry();
  }

  /**
   * Init when setting up teleop setting.
   */
  @Override
  public void teleopInit() {
    m_swerve.customAutoAlign();
    ballFondler.resetSystem();
  }

  /**
   * Init when setting up teleop setting.
   */
  @Override
  public void teleopPeriodic() {

    driveWithJoystick(true);
    handleInputs(xController, j_operator);
  }

  private void driveWithJoystick(boolean fieldRelative) {
    /**
     * Get desired X speed of chassis.
     * Inverted since Xbox joysticks return flipped values.
     * 
     **/
    final double xSpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband(xController.getLeftY(), 0.08))
        * frc.robot.swerveCode.Drivetrain.kMaxSpeed;
    /**
     * Get desired Y (strafe/sideways) speed of chassis.
     * Positive = left, negative = right.
     * XBox controllers return flipped values.
     **/
    final double ySpeed = -m_yspeedLimiter.calculate(MathUtil.applyDeadband(xController.getLeftX(), 0.08))
        * frc.robot.swerveCode.Drivetrain.kMaxSpeed;
    /**
     * Get desired rotation speed of chassis.
     * Positive = left, negative = right.
     * Xbox returns positive when holding right by default.
     **/
    final double rot = -m_rotLimiter.calculate(MathUtil.applyDeadband(xController.getRightX(), 0.12))
        * frc.robot.swerveCode.Drivetrain.kMaxAngularSpeed;
      
    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
  }

  private void handleInputs(XboxController xController, Joystick j_operator) {
    ballFondler.handleInputs(xController, j_operator);
    climbModule.handleInputs(xController, j_operator);
  }


  private void resetSystem() {
    ballFondler.resetSystem();
    climbModule.resetSystem();
  }
}
