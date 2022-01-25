package frc.robot;

//sparkmax/neos imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;

/**
 * Intake class that has event for running
 */
public class Intake implements Runnable {
    
    //neos
    private CANSparkMax m_Intake = new CANSparkMax(-1, MotorType.kBrushless); //negative power for in, positive power for out //OG 6

    // Ball indexer variable
    public int ballIndexer = 0;

    // Limit switch
    private DigitalInput indexLimitSwitch = new DigitalInput(0); // Plug into DIO channel 0

    // solenoid variables
    private Solenoid s_LeftIntake = new Solenoid(null, -1);
    private Solenoid s_RightIntake = new Solenoid(null, -1);

    // logic variables
    public boolean intakeOn = false;
    private boolean intakeExtended = false;

    //functions
    public void extendIntake(){
        s_LeftIntake.set(true);
        s_RightIntake.set(true);
    }

    public void retractIntake() {
        s_LeftIntake.set(false);
        s_RightIntake.set(false);
    }

    public void ejectIntake() {
        if(intakeExtended) {
            m_Intake.set(-1);
            intakeOn = true;
            ballIndexer  -= 1;
        }
    }

    public boolean intake(boolean intakeOn){ //toggle intake motors
        
        if(intakeExtended == true){

            if (intakeOn == false){
                m_Intake.set(.3);
                intakeOn = true;
            }
            else{
                m_Intake.set(0);
                intakeOn = false;
            }
        }

        return intakeOn;
    }

    public void run() {
        intake(intakeOn);
        if (intakeOn && indexLimitSwitch.get()) {
            ballIndexer++;
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
