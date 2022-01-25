package frc.robot;

//sparkmax/neos imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

/**
 * Intake class that has event for running
 */
public class Intake implements Runnable {
    
    //neos
    private CANSparkMax m_Intake; //negative power for in, positive power for out //OG 6

    // Ball indexer variable
    public int ballIndexer = 0;

    // Limit switch
    private DigitalInput indexLimitSwitch; // Plug into DIO channel 0

    // solenoid variables
    private Solenoid s_LeftIntake;
    private Solenoid s_RightIntake;

    // logic variables
    public boolean intakeOn = false;
    private boolean intakeExtended = false;

    /**
     * Constructor for Intake Object
     * @param intakeChannel deviceId for Motor Controller used on intake
     * @param solenoidChannelLeftIntake Channel for left side pnuematic
     * @param solenoidChannelRightIntake Channel for right side pnuematic
     * @param limitSwitchChannel Channle for limit switch (DIO channel)
     */
    public Intake(int intakeId, int solenoidChannelLeftIntake, int solenoidChannelRightIntake, int limitSwitchChannel) {
        m_Intake = new CANSparkMax(intakeId, MotorType.kBrushless);
        s_LeftIntake = new Solenoid(PneumaticsModuleType.REVPH, solenoidChannelLeftIntake);
        s_RightIntake = new Solenoid(PneumaticsModuleType.REVPH, solenoidChannelRightIntake);
        indexLimitSwitch = new DigitalInput(limitSwitchChannel);
    }

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
