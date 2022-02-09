package frc.robot;

// imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
//wpilib imports
import edu.wpi.first.wpilibj.Solenoid;

public class Climb {

    // neos
    public CANSparkMax m_Hook = new CANSparkMax(-1, MotorType.kBrushless);
    public CANSparkMax m_Winch = new CANSparkMax(-1, MotorType.kBrushless);
    
    // solenoids
    public Solenoid s_LeftIntake = new Solenoid(null, -1);

    // sensors
    public DigitalInput armExtended = new DigitalInput(-1);
    public DigitalInput armClosed = new DigitalInput(-1);
    

    
}
