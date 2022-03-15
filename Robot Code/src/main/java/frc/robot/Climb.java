package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// imports
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
//wpilib imports
import edu.wpi.first.wpilibj.XboxController;
public class Climb {

    // // neos
    // public CANSparkMax m_Hook = new CANSparkMax(0, MotorType.kBrushless);
    public CANSparkMax m_Winch = new CANSparkMax(10, MotorType.kBrushless);

    // // neo encoders
    // public RelativeEncoder e_Hook = m_Hook.getEncoder();
    //public RelativeEncoder e_Winch = m_Winch.getEncoder();

    // // neo pidcontrollers
    // public SparkMaxPIDController pc_Hook = m_Hook.getPIDController();
    // public SparkMaxPIDController pc_Winch = m_Winch.getPIDController();

    // solenoids
    public Solenoid s_LeftPopArm = new Solenoid(PneumaticsModuleType.CTREPCM, 7);
    // public Solenoid s_RightPopArm = new Solenoid(PneumaticsModuleType.CTREPCM, 5);
    // public Solenoid s_RightPopArm = new Solenoid(PneumaticsModuleType.CTREPCM, 5);

    public Solenoid[] solenoids = new Solenoid[] { s_LeftPopArm };

    // sensors
    // return true if the circuit is open
    // public DigitalInput armNotExtended = new DigitalInput(0);
    // public DigitalInput armNotClosed = new DigitalInput(-1);

    // functions

    public void handleInputs(Joystick j_operator) {
        if (j_operator.getTrigger()) {
            extendAll();
        }
        if (j_operator.getRawButton(3)) {
            retractAll();
        }
        if (j_operator.getRawButton(4)) {
            extendWinch();
        } 
        if (j_operator.getRawButton(5)) {
            retractWinch();
        } 
    }

    
    public void extendAll() {
        s_LeftPopArm.set(true);
        // s_RightPopArm.set(true);

    }

    
    public void retractAll() {
        s_LeftPopArm.set(false);
        // s_RightPopArm.set(false);
    }

    
    public void toggleAll() {
        s_LeftPopArm.toggle();
        // s_RightPopArm.toggle();

    }

    public void extendWinch() {
        m_Winch.set(-0.5);
    }

    public void retractWinch() {
        m_Winch.set(-0.5);
    }

    
    public Solenoid[] getAllSolenoids() {
        return solenoids;
    }

    // latch onto pole
    /*
     * public void latchClimb() {
     * if (!armNotExtended.get()) { // if arm is not fully extended
     * pc_Winch.setReference(1, ControlType.kPosition);
     * s_LeftPopArm.set(true);
     * s_RightPopArm.set(true);
     * 
     * }
     * // } else {
     * // pc_Hook.setReference(-1, ControlType.kPosition);
     * // return false;
     * 
     * // }
     * //return true;
     * }
     */

    /*
     * // pull up to pole
     * public void pullClimb() {
     * if (!armNotClosed.get()) { // if arm is not closed
     * pc_Winch.setReference(-1, ControlType.kPosition);
     * s_LeftPopArm.set(false);
     * s_RightPopArm.set(false);
     * }
     * 
     * // } else {
     * // pc_Hook.setReference(1, ControlType.kPosition);
     * // return true;
     * 
     * // }
     * // return false;
     * }
     */

    // public void retractArm() {
    // pc_Winch.setReference(300.14, ControlType.kPosition);
    // }

    // public void extendArm() {
    // pc_Winch.setReference(3.14, ControlType.kPosition);
    // }

    // public void retractFrontHooks() {
    // pc_Hook.setReference(3.14, ControlType.kPosition);
    // }

    // public void engageFrontHooks() {
    // pc_Hook.setReference(3.14, ControlType.kPosition);
    // }

}
