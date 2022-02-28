// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;

import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  public Joystick j_ClimbOperator = new Joystick(2);
  private boolean killToggleOn = false;

  // climb variables
  private Climb climb = new Climb();
  private boolean frontHooksEngaged = false;
  private boolean armExtended = false;
  private boolean armPoppedUp = false;

  private int climbCounter = 0; // reaching 1 is high, 2 is traversal

  // PID variables
  public double kP_Winch = 0.0001;
  public double kI_Winch = 0;
  public double kD_Winch = 0.01;
  public double kIz_Winch = 0;
  public double kFF_Winch = 0.0001746724891;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    climb.pc_Winch.setP(kP_Winch);
    climb.pc_Winch.setI(kI_Winch);
    climb.pc_Winch.setD(kD_Winch);
    climb.pc_Winch.setIZone(kIz_Winch);
    climb.pc_Winch.setFF(kFF_Winch);
    
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    climb.m_Hook.setIdleMode(CANSparkMax.IdleMode.kBrake);
    climb.m_Winch.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    climb.popArmUp();

   
    // climb.retractArm();

    // // kill switch only stops future methods from being run
    // if (j_ClimbOperator.getRawButton(1)){
    //   if (killToggleOn) {
    //     killToggleOn = false;
    //   } else {
    //     killToggleOn = true;
    //   }
    // }

    // // winch toggle
    // if (j_ClimbOperator.getRawButton(4) && !killToggleOn){
    //   if (armExtended) {
    //     climb.retractArm();
    //     armExtended = false;
    //   } else {
    //     climb.extendArm();
    //     armExtended = true;
    //   }
    // }

    // // front hooks toggle
    // if (j_ClimbOperator.getRawButton(5) && !killToggleOn){
    //   if (frontHooksEngaged) {
    //     climb.retractFrontHooks();
    //     frontHooksEngaged = false;
    //   } else {
    //     climb.engageFrontHooks();
    //     frontHooksEngaged = false;
    //   }
    // }

    // // pneumatic popup toggle
    // if (j_ClimbOperator.getRawButton(6) && !killToggleOn){
    //   if (armPoppedUp) {
    //     climb.popArmDown();
    //     armPoppedUp = false;
    //   } else {
    //     climb.popArmUp();
    //     armPoppedUp = true;
    //   }

    // }

    // // latch
    // if (j_ClimbOperator.getRawButton(2) && !killToggleOn){
    //   climb.latchClimb();
    // }
  
    // // winch
    // if (j_ClimbOperator.getRawButton(3) && !killToggleOn){
    //   climb.pullClimb();
    // }

    




    // if (frontHooksEngaged && climbCounter > 3) {
    //   frontHooksEngaged = climb.latchClimb();

    // } else {
    //   frontHooksEngaged = climb.pullClimb();

    //   if(frontHooksEngaged) {
    //     climbCounter++;
    //   }
      
    // }


  }
}
