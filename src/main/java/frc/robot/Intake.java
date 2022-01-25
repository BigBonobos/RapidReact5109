package frc.robot;

//sparkmax/neos imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//wpilib imports
import edu.wpi.first.wpilibj.Solenoid;

public class Intake {
    
    //neos
    public CANSparkMax m_Intake = new CANSparkMax(-1, MotorType.kBrushless); //negative power for in, positive power for out //OG 6

    // Ball indexer variable
    public int ballIndexer = 0;

    //neo encoders
    public RelativeEncoder e_Intake = m_Intake.getEncoder(); //negative when intaking

    //neo pidcontrollers
    public SparkMaxPIDController pc_Intake = m_Intake.getPIDController();

    //solenoid variables
    public Solenoid s_LeftIntake = new Solenoid(null, -1);
    public Solenoid s_RightIntake = new Solenoid(null, -1);

    //logic variables
    public boolean intakeOn = false;
    public boolean intakeExtended = false;

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

                // Await sensors here
                ballIndexer += 1;
            }
            else{
                m_Intake.set(0);
                intakeOn = false;
            }
        }

        return intakeOn;
    }
}
