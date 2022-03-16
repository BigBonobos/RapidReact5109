package frc.robot.ballSys;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.controllers.BaseController;

public class Shooter implements BaseController {

    /**
     * This may be unneeded. This is only needed if we need to wait w/o blocking
     * main thread.
     * <p>
     * I am not sure of the functionality of Thread.sleep()
     * inside the lambda I supplied for .shoot()
     * </p>
     */
    @SuppressWarnings("unused")
    private static final ExecutorService threadPool = Executors.newCachedThreadPool();

    /**
     * This is the motor that actually shoots the ball (the bottom one).
     */
    private final CANSparkMax m_shooterMotor;

    /**
     * This is the motor that pushes the balls towards the propulsion once its up to
     * speed.
     * 
     * <p>
     * CONTROLS:
     * 
     * <pre>
     *Assuming values > 0 = forward, towards propulsion.
     *Assuming values < 0 = backward, away from propulsion.
     * </pre>
     * </p>
     */
    private final CANSparkMax m_indexMotor;

    /**
     * This is the encoder used to know the current velocity of the propulsion
     * motor.
     */
    private final RelativeEncoder m_shooterEncoder;

    /**
     * This is an unused encoder, would know info on feed/kickFoward motor.
     * Irrelevant.
     */
    @SuppressWarnings("unused")
    private final RelativeEncoder m_indexEncoder;

    /**
     * This controls the speed of the rev'ing motors.
     * <p>
     * This is used to accelerate the wheels to a certain point.
     * </p>
     * <p>
     * We cannot use it to SLOW THE MOTOR DOWN. Only SPEED UP TO/PAST A POINT.
     * </p>
     * <p>
     * This will be used to get assured shot distance. WE MAY OVERSHOOT.
     * </p>
     * <p>
     * I will adjust this later.
     * </p>
     */
    private final BangBangController overSpeedController;

    /**
     * Wanted RPM of the wheel.
     * <p>
     * IMO, should be decided differently.
     * Perhaps an approx. of m/s the ball leaves the shooter?
     * Then we can do predictive calculations. Much better.
     * For now, we do this (deprecated).
     * 
     * @deprecated
     */
    @SuppressWarnings("unused")
    private double shooterRpms;

    private final Notifier shootHandler;


    public int ballCount = 0;
    public double shooterRPMs;


    public Shooter(int m_indexPort, int m_shooterPort, double shooterSpeedRpms) {
        m_indexMotor = new CANSparkMax(m_indexPort, MotorType.kBrushless);
        m_shooterMotor = new CANSparkMax(m_shooterPort, MotorType.kBrushless);

        m_indexEncoder = m_indexMotor.getEncoder();
        m_shooterEncoder = m_shooterMotor.getEncoder();

        overSpeedController = new BangBangController(50);

        m_shooterMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        overSpeedController.setSetpoint(shooterSpeedRpms);

        shootHandler = new Notifier(this::_newShoot);

        shooterRPMs = shooterSpeedRpms;

    }

    public double getWantedRPM() {
        return overSpeedController.getSetpoint();
    }

    public void setWantedRPM(double rpm) {
        shooterRPMs = rpm;
        overSpeedController.setSetpoint(rpm);
    }

    public boolean isAtShootingSpeed() {
        return overSpeedController.atSetpoint();
    }

    public double calculateShootingSpeed() {
        return overSpeedController.calculate(m_shooterEncoder.getVelocity(), this.getWantedRPM());
    }

    /**
     * @return velocity of motor is less than 20 rpm AND wanted speed of motor is NOT zero. 
     */
    public boolean isShooting() {
        return m_shooterEncoder.getVelocity() < 20 && m_shooterMotor.get() != 0;
    }

    /**
     * Since this is used in a notifier, System.out.println does not work as intended.
     * This is because it is being ran in a different thread than the RoboRIO's output.
     */
    private void _newShoot() {

        setWantedRPM(shooterRPMs);

        double ballDelay = 0.25;
        ballDelay = ballCount * 0.25;


        // if (!Beam1.get()) {
        //     ballDelay = 0.5;
        // }
        // if (!Beam2.get() && Beam1.get()) {
        //     ballDelay = 0.25;
        // }
        // if (Beam1.get() && Beam2.get()) {
        //     ballDelay = 0.25;
        // }

        m_indexMotor.set(0);
        double startTime = Timer.getFPGATimestamp();
        while (!isAtShootingSpeed() && Timer.getFPGATimestamp() - startTime < 4) {
            m_shooterMotor.set(calculateShootingSpeed());

            // System.out.printf("Ready?: %b   Speed: %f\n", isAtShootingSpeed(), m_shooterEncoder.getVelocity());
            // pause for ten milliseconds to not be wasteful.
            // if interrupted, break.
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                break;
            }
        }

      
        double loadingStart = Timer.getFPGATimestamp();
        while (Timer.getFPGATimestamp() - loadingStart < ballDelay) {
            System.out.println("We're loading teh balls now.");
            m_indexMotor.set(0.4);
            m_shooterMotor.set(calculateShootingSpeed());
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                return;
            }
        }

        m_indexMotor.set(0);
        m_shooterMotor.set(0);

    }

    public void newShoot() {
        shootHandler.startSingle(0.01);
    }

    @Override
    public void handleInputs(XboxController xController, Joystick j_operator) {
        //Manual windup for testing.
        if (xController.getLeftBumper()) {
            m_shooterMotor.set(calculateShootingSpeed());
        } else {
            m_shooterMotor.set(0);
        }

    }

    @Override
    public void resetSystem() {
        m_indexMotor.set(0);
        m_shooterMotor.set(0);

    }

}