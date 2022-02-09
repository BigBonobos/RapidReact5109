    package frc.robot;

    import com.revrobotics.*;
    import com.revrobotics.CANSparkMaxLowLevel.MotorType;

    import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
    import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
    import edu.wpi.first.wpilibj.*;

    public class BallSystems {

        public Joystick j_Operator = new Joystick(1);

        //Sensors
        private DigitalInput Beam1 = new DigitalInput(0);
        private DigitalInput Beam2 = new DigitalInput(1);  

        //Variables
        private int BallCount;
        public boolean intakeOn = false; 
        private boolean intakeExtended = false; 
        
        //Motor
        private CANSparkMax m_indexWheel = new CANSparkMax(23, MotorType.kBrushless);
        private CANSparkMax m_shooterWheel = new CANSparkMax(24, MotorType.kBrushless);
        private CANSparkMax m_intakeWheel = new CANSparkMax(25, MotorType.kBrushless); 

        //Solenoids
        private Solenoid s_LeftIntake = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
        private Solenoid s_RightIntake = new Solenoid(PneumaticsModuleType.CTREPCM, 2);
        //private Solenoid s_ControlPanel = new Solenoid(PneumaticsModuleType.CTREPCM, 4);


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
                if (!Beam1.get()){
                    BallCount = 2; 
                }
            }
        }

        public void Shooting (){
            if (j_Operator.getTrigger()){
                m_indexWheel.set(0.3);
                m_shooterWheel.set(0.5);
                Timer.delay(3);
                m_indexWheel.stopMotor();
                m_shooterWheel.stopMotor();
                BallCount = 0; 
            }
        }

        public void extendIntake(){
            s_LeftIntake.set(true);
            s_RightIntake.set(true);
        }

        public void retractIntake(){
            s_LeftIntake.set(false);
            s_RightIntake.set(false);
        }

        public void ejectIntake() {
            if(intakeExtended) {
                m_intakeWheel.set(-1);
                intakeOn = true;
            }
        }

        public boolean intake(boolean intakeOn){
            if(intakeExtended == true){
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

