package frc.robot;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Dynamic;
import com.revrobotics.*;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.*;

public class BallSys { 

    //Sensors
    public DigitalInput Beam1 = new DigitalInput(0); //Outside the kicker wheel
    public DigitalInput Beam2 = new DigitalInput(1); //Inside the kicker wheel

    //Variables
    public int BallCount; 
    public boolean shooting;
    public boolean intakeOn; 
    public boolean intakeExtend;
    public boolean BoolBall;

    //Motor
    private CANSparkMax m_indexWheel = new CANSparkMax(23, MotorType.kBrushless);
    private CANSparkMax m_shooterWheel = new CANSparkMax(24, MotorType.kBrushless);
    private CANSparkMax m_intakeWheel = new CANSparkMax(25, MotorType.kBrushless); 

    //Solenoids
    private Solenoid s_LeftIntake = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
    private Solenoid s_RightIntake = new Solenoid(PneumaticsModuleType.CTREPCM, 2);

    //Encoders
    public RelativeEncoder e_shooterWheel = m_shooterWheel.getEncoder();
    public RelativeEncoder e_indexWheel = m_indexWheel.getEncoder();
    private SparkMaxPIDController p_indexWheel = m_indexWheel.getPIDController();

    //Controllers
    private BangBangController overSpeedController; 
    private int shooterRPMs = 1000; 


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
            }
        }
      }
    
    public void Shooting1() { 
        if (BallCount > 0){
            shooting = true; 
            m_shooterWheel.set(overSpeedController.calculate(e_shooterWheel.getVelocity(), shooterRPMs));
            if (e_shooterWheel.getVelocity() == shooterRPMs && e_indexWheel.getPosition() <= 1){
                m_indexWheel.set(0.4);
                BallCount --;
            }
            else{
              m_indexWheel.stopMotor();
              e_indexWheel.setPosition(0);
            }
        }
        else{
            m_shooterWheel.stopMotor();
            shooting = false;
        }
    }

    public void Shooting2() {

        if (BallCount > 0){
            shooting = true; 
            m_shooterWheel.set(overSpeedController.calculate(e_shooterWheel.getVelocity(), shooterRPMs));
            if (e_shooterWheel.getVelocity() == shooterRPMs && !Beam2.get()){
                m_indexWheel.set(0.4);
                BoolBall = true;
            }
            else{
                m_indexWheel.stopMotor();
                if (Beam2.get() && BoolBall == true){
                    BallCount --; 
                    BoolBall = false;
                }
            }
        }
        else{
          m_indexWheel.stopMotor();
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
            }
            else{
                m_intakeWheel.stopMotor();
            }
        //}
        return intakeOn;
      }
    }
