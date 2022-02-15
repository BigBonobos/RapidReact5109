package frc.robot;

// imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
//wpilib imports
import edu.wpi.first.wpilibj.Solenoid;

public class Climb {

    // neos
    public CANSparkMax m_Hook = new CANSparkMax(-1, MotorType.kBrushless);
    public CANSparkMax m_Winch = new CANSparkMax(-1, MotorType.kBrushless);

    // neo encoders
    public RelativeEncoder e_Hook = m_Hook.getEncoder();
    public RelativeEncoder e_Winch = m_Winch.getEncoder();

    // neo pidcontrollers
    public SparkMaxPIDController pc_Hook = m_Hook.getPIDController();
    public SparkMaxPIDController pc_Winch = m_Winch.getPIDController();
    
    // solenoids
    public Solenoid s_PopArm = new Solenoid(null, -1);

    // sensors
    // return true if the circuit is open
    public DigitalInput armNotExtended = new DigitalInput(-1);
    public DigitalInput armNotClosed = new DigitalInput(-1);

    // functions

    // latch onto pole (will not work)
    public boolean latchClimb() {
        if (!armNotExtended.get()) {
            pc_Winch.setReference(-1, ControlType.kPosition);
            // s_PopArm.set(true);

        } else {
            pc_Hook.setReference(-1, ControlType.kPosition);
            return false;

        }
        return true;
    }

    // pull up to pole (will not work)
    public boolean pullClimb() {
        if (!armNotClosed.get()) {
            pc_Winch.setReference(-1, ControlType.kPosition);

        } else {
            pc_Hook.setReference(-1, ControlType.kPosition);
            return true;

        }
        return false;
    }

    public void retractArm() {
        pc_Winch.setReference(-1, ControlType.kPosition);
    }

    public void extendArm() {
        pc_Winch.setReference(-1, ControlType.kPosition);
    }

    public void retractFrontHooks() {
        pc_Hook.setReference(-1, ControlType.kPosition);
    }

    public void engageFrontHooks() {
        pc_Hook.setReference(-1, ControlType.kPosition);
    }

    public void popArmUp() {
        s_PopArm.set(true);
    }

    public void popArmDown() {
        s_PopArm.set(false);
    }

}
