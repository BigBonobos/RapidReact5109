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

  //Methods
  BallSys BS = new BallSys();
  
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
    SmartDashboard.putNumber("BallConut", BallCount);
    SmartDashboard.putBoolean("IntakeMotor", intakeOn);
    SmartDashboard.putBoolean("Shooting", shooting); 

    BS.Index();

    /*if (xBox.getAButton()){
      intakeSolenoid(intakeExtend = !intakeExtend);
    }*/

    if (xBox.getLeftBumper()){
      BS.intakeMotor(intakeOn = !intakeOn);
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

    //put shooter on coats, and index wheel on brake

  }


  
