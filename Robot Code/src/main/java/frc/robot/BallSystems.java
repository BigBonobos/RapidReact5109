package frc.robot;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Dynamic;
import com.revrobotics.*;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.*;

public class BallSystems implements Runnable { 

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
    public CANSparkMax m_indexWheel = new CANSparkMax(22, MotorType.kBrushless); //correct
    public CANSparkMax m_shooterWheel = new CANSparkMax(8, MotorType.kBrushless); //correct
    public CANSparkMax m_intakeWheel = new CANSparkMax(4, MotorType.kBrushless); 

    //Solenoids
    private Solenoid s_LeftIntake = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
    private Solenoid s_RightIntake = new Solenoid(PneumaticsModuleType.CTREPCM, 2);

    //Encoders
    public RelativeEncoder e_shooterWheel = m_shooterWheel.getEncoder();
    public RelativeEncoder e_indexWheel = m_indexWheel.getEncoder();
    private SparkMaxPIDController p_indexWheel = m_indexWheel.getPIDController();

    //Controllers
    public BangBangController overSpeedController; 
    public int shooterRPMs = 2000; 


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
    
    public boolean shooting2(boolean shooting) {
        while (shooting == true){
            if (e_shooterWheel.getVelocity() > 2700){
                m_shooterWheel.set(0.4);
            }
            if (e_shooterWheel.getVelocity() < 2600){
                m_shooterWheel.set(0.6);
            }

            if (BallCount > 0){
                if (Beam2.get() || e_shooterWheel.getVelocity() >= 2500){
                    m_indexWheel.set(0.3);
                } else {
                    m_indexWheel.stopMotor();
                }
            }

            if (!Beam2.get()){
                BoolBall = true;
            }
            if (Beam2.get()){
                BoolBall = false;
                if (BoolBall = false){
                    BallCount --;
                }
            }
            
            if (BallCount == 0){
                m_indexWheel.stopMotor();
                m_shooterWheel.stopMotor();
                shooting = false;
            }
        }
        
        // else{
        //     m_shooterWheel.set(0);
        //     m_indexWheel.stopMotor();
        //     shooting = false; 
        // }
        return shooting;
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
    
    /*public boolean intakeMotor(boolean intakeOn){
        //if (intakeExtend == true)
            if(intakeOn == true){
                m_intakeWheel.set(0.3);
            }
            else{
                m_intakeWheel.stopMotor();
            }
        //}
        return intakeOn;
    }*/
    public void intakeMotor(){
        if (shooting == false && BallCount < 2){
            m_intakeWheel.set(0.75);
        }
        else {
            m_intakeWheel.stopMotor();
        }
    }

    @Override
    public void run() {
        shooting2(true);
    }
}
