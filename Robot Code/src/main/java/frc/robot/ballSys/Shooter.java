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
    private final CANSparkMax shooterPropulsionMotor;

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
    private final CANSparkMax shooterKickForwardMotor;

    /**
     * This is the encoder used to know the current velocity of the propulsion
     * motor.
     */
    private final RelativeEncoder shooterPropulsionEncoder;

    /**
     * This is an unused encoder, would know info on feed/kickFoward motor.
     * Irrelevant.
     */
    @SuppressWarnings("unused")
    private final RelativeEncoder shooterKickForwardEncoder;

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
    private int shooterRpms;

    public DigitalInput Beam1 = new DigitalInput(0); // Outside the kicker wheel
    public DigitalInput Beam2 = new DigitalInput(1); // Inside the kicker wheel

    private final Notifier shootHandler;
    private int BallDelay;

    private int shooterRPMs;

    public Shooter(int m_indexPort, int m_shooterPort, int shooterSpeedRpms) {
        shooterKickForwardMotor = new CANSparkMax(m_indexPort, MotorType.kBrushless);
        shooterPropulsionMotor = new CANSparkMax(m_shooterPort, MotorType.kBrushless);

        shooterKickForwardEncoder = shooterKickForwardMotor.getEncoder();
        shooterPropulsionEncoder = shooterPropulsionMotor.getEncoder();

        overSpeedController = new BangBangController(50);

        shooterPropulsionMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        overSpeedController.setSetpoint(shooterSpeedRpms);

        shootHandler = new Notifier(this::_newShoot);

        shooterRPMs = shooterSpeedRpms;

    }

    public double getWantedRPM() {
        return overSpeedController.getSetpoint();
    }

    public void setWantedRPM(double rpm) {
        overSpeedController.setSetpoint(rpm);
    }

    public boolean isAtShootingSpeed() {
        return overSpeedController.atSetpoint();
    }

    public double calculateShootingSpeed() {
        return overSpeedController.calculate(shooterPropulsionEncoder.getVelocity(), this.getWantedRPM());
    }

    private void _newShoot() {

        setWantedRPM(shooterRPMs);

        double ballDelay = 0.25;
        if (!Beam1.get()) {
            ballDelay = 0.5;
        }
        if (!Beam2.get() && Beam1.get()) {
            ballDelay = 0.25;
        }
        if (Beam1.get() && Beam2.get()) {
            ballDelay = 0.25;
        }

        shooterKickForwardMotor.set(0);
        double startTime = Timer.getFPGATimestamp();
        System.out.printf("Ready?: %b\n", isAtShootingSpeed());
        while (!isAtShootingSpeed() && Timer.getFPGATimestamp() - startTime < 4) {
            shooterPropulsionMotor.set(calculateShootingSpeed());

            System.out.printf("Ready?: %b   Speed: %f\n", isAtShootingSpeed(), shooterPropulsionEncoder.getVelocity());
            // pause for ten milliseconds to not be wasteful.
            // if interrupted, break.
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                break;
            }
        }

        System.out.printf("Ready?: %b   Speed: %f\n", isAtShootingSpeed(), shooterPropulsionEncoder.getVelocity());

        double loadingStart = Timer.getFPGATimestamp();
        System.out.printf("Time delay: %f\n", ballDelay);
        while (Timer.getFPGATimestamp() - loadingStart < ballDelay) {
            System.out.println("We're loading teh balls now.");
            shooterKickForwardMotor.set(0.4);
            shooterPropulsionMotor.set(calculateShootingSpeed());
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                return;
            }
        }

        shooterKickForwardMotor.set(0);
        shooterPropulsionMotor.set(0);

    }

    public void newShoot() {
        shootHandler.startSingle(0.01);

    }

    @Override
    public void handleInputs(XboxController xController, Joystick j_operator) {
        if (xController.getRightTriggerAxis() == 1) {
            newShoot();
        }

        if (xController.getLeftBumper()) {
            shooterPropulsionMotor.set(calculateShootingSpeed());
        } else {
            shooterPropulsionMotor.set(0);
        }

    }

    @Override
    public void resetSystem() {
        // TODO Auto-generated method stub

    }

}