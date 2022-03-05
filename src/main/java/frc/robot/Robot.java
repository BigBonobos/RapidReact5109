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
    BS.BoolBall = false; 
    //BS.BallCount = 2; 
    BS.shooting = false;
  }

  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("BallCount", BS.BallCount);
    SmartDashboard.putBoolean("Shooting", BS.shooting); 
    SmartDashboard.putBoolean("IntakeRunning", BS.intakeOn);
    SmartDashboard.putBoolean("Beam1", BS.Beam1.get());
    SmartDashboard.putBoolean("Beam2", BS.Beam2.get());
    SmartDashboard.putNumber("RPMs", BS.e_shooterWheel.getVelocity());

    //BS.m_indexWheel.set(0.1);
    BS.Index();
    BS.intakeMotor();
    //BS.m_intakeWheel.set(0.3);

    /*if (xBox.getAButton()){
      intakeSolenoid(intakeExtend = !intakeExtend);
    }*/

    // if (xBox.getLeftBumper()){
    //     BS.intakeMotor(BS.intakeOn = !BS.intakeOn);
    //   }

    /*if (xBox.getRightBumper()){
      BS.Shooting1();
    }*/

    if (xBox.getRightTriggerAxis() == 1){
      BS.Shooting2(true);
      //xBox.setRumble(RumbleType.kRightRumble, 1);
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
    
    SmartDashboard.putBoolean("BeamBreak1", BS.Beam1.get());
    SmartDashboard.putBoolean("BeamBreak2", BS.Beam2.get());
    SmartDashboard.putNumber("IndexEncoders", BS.e_indexWheel.getPosition());
    //Orientation of Beam Breaks
    BS.Beam2.get();
    BS.Beam1.get();

    //Orientation of Shooter; ++ is shoot
    //m_shooterWheel.set(0.3);
    //Orientation of Intake; ++ is intake
    //m_intakeWheel.set(0.3);
    //Orientation of index Motor; ++ is intake
    //m_indexWheel.set(0.3);

    //Determing how many encoders = 1 ball shot
    /*if (xBox.getLeftBumper()){
      m_indexWheel.set(0.2);
    }*/

    //put shooter on coats, and index wheel on brake

  }
}
