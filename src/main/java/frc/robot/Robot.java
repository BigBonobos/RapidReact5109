// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  
  //Controllers
  public XboxController xBox = new XboxController(1);
  public Joystick j_Operator = new Joystick(1);

  //Sensors
  public DigitalInput Beam1 = new DigitalInput(0); //Outside the kicker wheel
  public DigitalInput Beam2 = new DigitalInput(1); //Inside the kicker wheel
  
  //Variables
  public int BallCount; 
  public boolean shooting;
  public boolean intakeOn = false; 
  public boolean intakeExtend;
  public boolean boolBeam2; 

  //Motor
  public CANSparkMax m_indexWheel = new CANSparkMax(23, MotorType.kBrushless);
  public CANSparkMax m_shooterWheel = new CANSparkMax(24, MotorType.kBrushless);
  public CANSparkMax m_intakeWheel = new CANSparkMax(25, MotorType.kBrushless); 

  //Solenoids
  public Solenoid s_LeftIntake = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
  public Solenoid s_RightIntake = new Solenoid(PneumaticsModuleType.CTREPCM, 2);

  //Encoders
  public RelativeEncoder e_shooterWheel = m_shooterWheel.getEncoder();
  public RelativeEncoder e_indexWheel = m_indexWheel.getEncoder();
  public SparkMaxPIDController p_indexWheel = m_indexWheel.getPIDController();

  //Controllers
  public BangBangController overSpeedController; 
  public int shooterRPMs = 1000; 
  
  @Override
  public void robotInit() {
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("BallConut", BallCount);
    SmartDashboard.putBoolean("IntakeMotor", intakeOn);
    SmartDashboard.putBoolean("Shooting", shooting); 
    Index();

    /*if (xBox.getAButton()){
      intakeSolenoid(intakeExtend = !intakeExtend);
    }*/

    if (xBox.getLeftBumper()){
      intakeMotor(intakeOn = !intakeOn);
    }

    if (xBox.getRightBumper()){
      Shooting();
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {
    e_indexWheel.setPosition(0);
  }

  @Override
  public void testPeriodic() {
    
    SmartDashboard.putBoolean("BeamBreak", boolBeam2);
    SmartDashboard.putNumber("IndexEncoders", e_indexWheel.getPosition());
    //Orientation of Beam Breaks
    if (Beam2.get()){
      boolBeam2 = false;
    }
    else{
      boolBeam2 = true; 
    }

    //Orientation of Shooter
    m_shooterWheel.set(0.3);
    //Orientation of Intake
    m_intakeWheel.set(0.3);
    //Orientation of index Motor
    m_indexWheel.set(0.3);

    //Determing how many encoders = 1 ball shot
    /*if (xBox.getLeftBumper()){
      m_indexWheel.set(0.2);
    }*/

  }


  public void Index() {
    if (shooting == false){
        if (BallCount == 0){
            if (!Beam1.get()){
                m_indexWheel.set(0.4);
            }
            if (!Beam2.get()){
                m_indexWheel.stopMotor();
                BallCount = 1; 
            }
        }
        if (BallCount == 1){
            if (!Beam1.get() && !Beam2.get()){
                m_indexWheel.stopMotor();
                BallCount = 2; 
            }
            if (!Beam1.get() && Beam2.get()){
                m_indexWheel.set(0.4);
            }
        }
    }
  }

  public void Shooting() { 
    if (BallCount > 0){
        shooting = true; 
        e_indexWheel.setPosition(0);
        m_shooterWheel.set(overSpeedController.calculate(e_shooterWheel.getVelocity(), shooterRPMs));
        if (e_shooterWheel.getVelocity() == shooterRPMs && e_indexWheel.getPosition() <= 1){
            m_indexWheel.set(0.4);
            BallCount --;
        }
        else{
          m_indexWheel.stopMotor();
        }
    }
    else{
        m_shooterWheel.stopMotor();
        shooting = false;
    }
   
  }

  /*public boolean intakeSolenoid(boolean intakeExtend){
    if (intakeExtend == false){
        s_LeftIntake.set(true);
        s_RightIntake.set(true);
        intakeExtend = true; 
    }
    if (intakeExtend == true){
        s_LeftIntake.set(false);
        s_RightIntake.set(false);
        intakeExtend = false; 
    }
    return intakeExtend;
  } */

  public boolean intakeMotor(boolean intakeOn){
    //if (intakeExtend == true){
        if(intakeOn == false){
            m_intakeWheel.set(0.3);
            intakeOn = true; 
        }
        else{
            m_intakeWheel.stopMotor();
            intakeOn = false; 
        }
    //}
    return intakeOn;
  }
}

