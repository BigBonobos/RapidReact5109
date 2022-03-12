package frc.robot.ballSys;

// sparkmax/neos imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

/**
 * Intake class that has event for running
 */
public class Intake {
    
    //neos
    private CANSparkMax m_Intake; //negative power for in, positive power for out //OG 6

    // Ball indexer variable
    public int ballIndexer = 0;

    // Limit switch
    private DigitalInput indexLimitSwitch; // Plug into DIO channel 0

    // logic variables
    public boolean intakeOn = false;
    private boolean intakeExtended = false;

    /**
     * Constructor for Intake Object
     * @param intakeChannel deviceId for Motor Controller used on intake
     * @param beamBreaker Channel for beam breaker (DIO channel)
     */
    public Intake(int intakeId, int beamBreaker) {
        m_Intake = new CANSparkMax(intakeId, MotorType.kBrushless);
        indexLimitSwitch = new DigitalInput(beamBreaker);
    }

    public void ejectIntake() {
        if(intakeExtended) {
            m_Intake.set(-1);
            intakeOn = true;
            ballIndexer  -= 1;
        }
    }

    public void checkIntakeState() {
        if (indexLimitSwitch.get()) {
            ballIndexer++;
            Timer.delay(1.5);
        }
    }

    public boolean intake(boolean intakeOn){ //toggle intake motors
        
        if(intakeExtended == true){
            if (intakeOn == false){
                m_Intake.set(.3);
                intakeOn = true;
            }
            else {
                m_Intake.set(0);
                intakeOn = false;
            }
        }

        return intakeOn;
    }

}
