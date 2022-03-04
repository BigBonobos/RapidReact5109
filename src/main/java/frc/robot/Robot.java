package frc.robot;

import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  
  //Controllers
  public XboxController xBox = new XboxController(1);
  public Joystick j_Operator = new Joystick(1);

  //Methods
  BallSys BS = new BallSys();

  //Motors for testing
  private CANSparkMax m_indexWheel = new CANSparkMax(23, MotorType.kBrushless);
  private CANSparkMax m_shooterWheel = new CANSparkMax(24, MotorType.kBrushless);
  private CANSparkMax m_intakeWheel = new CANSparkMax(25, MotorType.kBrushless); 
  
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
  public void teleopInit() {
    BS.intakeOn = false; 
  }

  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("BallConut", BS.BallCount);
    SmartDashboard.putBoolean("IntakeMotor", BS.intakeOn);
    SmartDashboard.putBoolean("Shooting", BS.shooting); 

    BS.Index();

    /*if (xBox.getAButton()){
      intakeSolenoid(intakeExtend = !intakeExtend);
    }*/

    if (xBox.getLeftBumper()){
      BS.intakeMotor(BS.intakeOn = !BS.intakeOn);
    }

    if (xBox.getRightBumper()){
      BS.Shooting1();
    }

    if (xBox.getRightTriggerAxis() != 0){
      BS.Shooting2();
      xBox.setRumble(RumbleType.kRightRumble, 1);
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {
    BS.e_indexWheel.setPosition(0);
  }

  @Override
  public void testPeriodic() {
    
    SmartDashboard.putBoolean("BeamBreak", BS.Beam2.get());
    SmartDashboard.putNumber("IndexEncoders", BS.e_indexWheel.getPosition());
    //Orientation of Beam Breaks

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

    //put shooter on coats, and index wheel on brake

  }


  
