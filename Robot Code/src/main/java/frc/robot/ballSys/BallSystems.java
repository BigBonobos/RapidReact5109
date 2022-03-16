package frc.robot.ballSys;

import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.*;
import frc.robot.controllers.BaseController;

public class BallSystems implements BaseController, Runnable {

    enum IntakeState {
        Stopped,
        Go,
        Reverse
    }

    public IntakeState intakeState = IntakeState.Stopped;

    // Sensors
    public DigitalInput Beam1 = new DigitalInput(0); // Outside the kicker wheel
    public DigitalInput Beam2 = new DigitalInput(1); // Inside the kicker wheel

    // Variables
    public int ballCount;
    public boolean shooting;
    public boolean intakeOn;
    public boolean intakeExtend;
    public boolean boolBall;

    // Motor
    public CANSparkMax m_indexWheel;
    public CANSparkMax m_shooterWheel;
    public CANSparkMax m_intakeWheel;

    // Solenoids
    @SuppressWarnings("unused")
    private Solenoid s_LeftIntake = new Solenoid(PneumaticsModuleType.CTREPCM, 1);

    @SuppressWarnings("unused")
    private Solenoid s_RightIntake = new Solenoid(PneumaticsModuleType.CTREPCM, 2);

    @SuppressWarnings("unused")
    private SparkMaxPIDController p_indexWheel;

    // Encoders
    public RelativeEncoder e_shooterWheel;
    public RelativeEncoder e_indexWheel;

    // Controllers
    public BangBangController overSpeedController;

    // private int minWantedRPMs;
    // private int maxWantedRPMs;

    /**
     * @param ports Required ports for the CANSparkMax motors.
     *              The order is:
     *              <p>
     *              Intake wheel (outside roller, collects balls into bot)
     *              <p>
     *              Index wheel (bottom wheel, moves balls up shaft) [lol]
     *              <p>
     *              Shooter wheel (top wheel, wheel that launches balls upward)
     *              [kek]
     */
    public BallSystems(int[] ports /* int speedUpToRPM, int slowDownAtRPM */) {
        if (ports.length != 3) {
            throw new IllegalArgumentException(String.format(
                    "Damn freshie, you fucked up hard.\nYou passed in %i motors when you need three.",
                    ports.length));

        }
        m_intakeWheel = new CANSparkMax(ports[0], MotorType.kBrushless);
        m_indexWheel = new CANSparkMax(ports[1], MotorType.kBrushless); // correct
        m_shooterWheel = new CANSparkMax(ports[2], MotorType.kBrushless); // correct

        p_indexWheel = m_indexWheel.getPIDController();

        e_shooterWheel = m_shooterWheel.getEncoder();
        e_indexWheel = m_indexWheel.getEncoder();
        // minWantedRPMs = speedUpToRPM;
        // maxWantedRPMs = slowDownAtRPM;
    }

    public void updateIndex() {
        if (shooting == false) {
            if (ballCount == 0) {
                if (!Beam1.get()) {
                    m_indexWheel.set(0.6);
                }
                if (!Beam2.get()) {
                    m_indexWheel.stopMotor();
                    ballCount = 1;
                }
            }
            if (ballCount == 1) {
                if (!Beam1.get() && !Beam2.get()) {
                    m_indexWheel.stopMotor();
                    ballCount = 2;
                }
            }
        }
    }

    public void windUpShooter() {
        if (e_shooterWheel.getVelocity() > 3800) {
            m_shooterWheel.set(0.6);
        } else if (e_shooterWheel.getVelocity() < 3500) {
            m_shooterWheel.set(0.8);
        }

    }

    public void stopShooter() {
        m_shooterWheel.set(0);
    }

    /**
     * Head's up, this is where our issue is. Lol.
     * We don't run this correctly.
     * 
     * <p>
     * Could throw an error here to enforce good code.
     * <p>
     * Will not, instead will just silently return.
     */
    public void newshoot2() {
        if (shooting)
            return;

        shooting = true;
        double startTime = Timer.getFPGATimestamp();
        double currentTime;
        while (true) {
            currentTime = Timer.getFPGATimestamp();
            if (e_shooterWheel.getVelocity() > 3500) {
                m_shooterWheel.set(0.6);
            } else if (e_shooterWheel.getVelocity() < 3400) {
                m_shooterWheel.set(0.8);
            }

            if (ballCount > 0) {
                if (e_shooterWheel.getVelocity() >= 3200) {
                    m_indexWheel.set(0.5);
                } else {
                    m_indexWheel.stopMotor();
                }

            }
            if (currentTime - startTime >= 4) {
                ballCount = 0;
                m_indexWheel.stopMotor();
                m_shooterWheel.stopMotor();
                shooting = false;
                break;
            }
        }

    }

    /**
     * For the time being, I'm leaving this code untouched. (Rocco)
     * 
     * <p>
     * UPDATE: Nvm, Xander is a retard. What the fuck is this variable assignment?
     * 
     */
    public boolean shooting2(boolean shooting) {
        double startTime = Timer.getFPGATimestamp();
        double currentTime = startTime;
        while (shooting == true) {
            currentTime = Timer.getFPGATimestamp();
            // System.out.println(e_shooterWheel.getVelocity());
            if (e_shooterWheel.getVelocity() > 3500) {
                m_shooterWheel.set(0.6);
            }
            if (e_shooterWheel.getVelocity() < 3400) {
                m_shooterWheel.set(0.8);
            }

            if (ballCount > 0) {
                if (e_shooterWheel.getVelocity() >= 3200) {
                    m_indexWheel.set(0.5);
                } else {
                    m_indexWheel.stopMotor();
                }
            }

            if (currentTime - startTime >= 4) {
                ballCount = 0;
                m_indexWheel.stopMotor();
                m_shooterWheel.stopMotor();
                shooting = false;
            }
        }
        return shooting;
    }

    public void intakeMotor() {
        if (shooting == false && ballCount < 2) {
            m_intakeWheel.set(0.75);
        } else {
            m_intakeWheel.stopMotor();
        }
    }

    public void handleInputs(XboxController xController, Joystick j_operator) {
        if (xController.getXButton()) {
            switch (intakeState) {
                case Stopped:
                    m_intakeWheel.set(0.4);
                    intakeState = IntakeState.Go;
                    break;
                case Go:
                    m_intakeWheel.set(-0.4);
                    intakeState = IntakeState.Reverse;
                case Reverse:
                    m_intakeWheel.set(0);
                    intakeState = IntakeState.Stopped;
            }
            m_intakeWheel.set(0.4);
        }

        if (xController.getXButton()) {
            m_indexWheel.set(0.4);
        } else {
            m_indexWheel.set(0);
        }

        if (xController.getYButton()) {
            // ballSysNotif.startSingle(0.0001);
            windUpShooter();
        } else {
            stopShooter();
        }

        if (xController.getLeftTriggerAxis() == 1) {
            m_intakeWheel.set(1);
        } else {
            m_intakeWheel.set(0);
        }

    }

    @Override
    public void resetSystem() {
        intakeOn = false;
        boolBall = false;
        shooting = false;
        ballCount = 0;
    }

    @Override
    public void run() {
        shooting2(true);
    }
}
