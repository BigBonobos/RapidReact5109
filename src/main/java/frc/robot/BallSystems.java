package frc.robot;

import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.*;

public class BS { 

    public Joystick j_Operator = new Joystick(1);

    //Sensors
    private DigitalInput Beam1 = new DigitalInput(0); //Outside the kicker wheel
    private DigitalInput Beam2 = new DigitalInput(1); //Inside the kicker wheel

    //Variables
    private int BallCount; 
    public boolean intakeOn = false; 
    public boolean intakeExtend; 
    public boolean newBall2; 
    
    //Motor
    private CANSparkMax m_indexWheel = new CANSparkMax(23, MotorType.kBrushless);
    private CANSparkMax m_shooterWheel = new CANSparkMax(24, MotorType.kBrushless);
    private CANSparkMax m_intakeWheel = new CANSparkMax(25, MotorType.kBrushless); 

    //Solenoids
    private Solenoid s_LeftIntake = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
    private Solenoid s_RightIntake = new Solenoid(PneumaticsModuleType.CTREPCM, 2);

    //Encoders
    private RelativeEncoder e_shooterWheel = m_shooterWheel.getEncoder();
    private RelativeEncoder e_indexWheel = m_indexWheel.getEncoder();

    //Controllers
    private BangBangController overSpeedController; 
    private int shooterRPMs; 


    public void Index() {
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

        if (Beam2.get()){
            newBall2 = true;
        }
        else{
            newBall2 = false;
        }
        
    }

    public void Shooting () { 
        if (BallCount > 0 ){
            m_shooterWheel.set(overSpeedController.calculate(e_shooterWheel.getVelocity(), shooterRPMs));
            if (e_shooterWheel.getVelocity() == shooterRPMs){
                m_indexWheel.set(0.4);
                if (newBall2 == true){
                    BallCount --; 
                }
            }
            else {
                m_indexWheel.stopMotor();
            }
        }
        else{
            m_shooterWheel.stopMotor();
        }
    }


    public boolean intakeSolenoid(boolean intakeExtend){
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
    }

    public boolean intakeMotor(boolean intakeOn){
        if(intakeExtend == true){
            if(intakeOn == false){
                m_intakeWheel.set(0.3);
                intakeOn = true; 
            }
            else{
                m_intakeWheel.stopMotor();
                intakeOn = false; 
            }
        }
        return intakeOn;
    }
 
}
