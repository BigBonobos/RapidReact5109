package frc.robot.shooter;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.BangBangController;

public class Shooter {

    /**
     * This may be unneeded. This is only needed if we need to wait w/o blocking main thread.
     * <p>I am not sure of the functionality of Thread.sleep() 
     * inside the lambda I supplied for .shoot()</p>
     */
    @SuppressWarnings("unused")
    private static final ExecutorService threadPool = Executors.newCachedThreadPool();

    /**
     * This is the motor that actually shoots the ball (the bottom one).
     */
    private final CANSparkMax shooterPropulsionMotor;

    /**
     * This is the motor that pushes the balls towards the propulsion once its up to speed.
     * 
     * <p>CONTROLS:
     * <pre>
     *Assuming values > 0 = forward, towards propulsion.
     *Assuming values < 0 = backward, away from propulsion.
     * </pre>
     * </p>
     */
    private final CANSparkMax shooterKickForwardMotor;

    /**
     * This is the encoder used to know the current velocity of the propulsion motor.
     */
    private final RelativeEncoder shooterPropulsionEncoder;

    /**
     * This is an unused encoder, would know info on feed/kickFoward motor. Irrelevant.
     */
    @SuppressWarnings("unused")
    private final RelativeEncoder shooterKickForwardEncoder;

    /**
     * This controls the speed of the rev'ing motors.
     * <p>This is used to accelerate the wheels to a certain point.</p>
     * <p>We cannot use it to SLOW THE MOTOR DOWN. Only SPEED UP TO/PAST A POINT.</p>
     * <p>This will be used to get assured shot distance. WE MAY OVERSHOOT.</p>
     * <p>I will adjust this later.</p>
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

    /**
     * Internal variable to cancel potentially multiple running events.
     * Prevents parallel calculations of propulsion motor.
     */
    private boolean _running;

    public Shooter(int propulsionPort, int kickForwardPort, int shooterSpeedRpms) {
        shooterKickForwardMotor = new CANSparkMax(kickForwardPort, MotorType.kBrushed);
        shooterPropulsionMotor = new CANSparkMax(propulsionPort, MotorType.kBrushed);

        shooterKickForwardEncoder = shooterKickForwardMotor.getEncoder();
        shooterPropulsionEncoder = shooterPropulsionMotor.getEncoder();

        overSpeedController = new BangBangController();

        shooterPropulsionMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        overSpeedController.setSetpoint(shooterSpeedRpms);

        _running = false;

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

    /**
     * I currently have no better way of doing this.
     * I am using a rough Thread.sleep() currently.
     * This halts the entire thread this code is being ran on.
     * Needless to say, this is terrible.
     * 
     * <p>Update: Made it async. SHOULD work. Untested.
     * I'll patch it up later if necessary.</p>
     * <p>
     * TODO: Implement async here. Make waits occur on separate thread.
     */
    public void shoot() {
        if (this._running)
            return;

        this._running = true;
        CompletableFuture.runAsync(() -> {
            while (!isAtShootingSpeed()) {
                shooterPropulsionMotor.set(calculateShootingSpeed());
                shooterKickForwardMotor.set(0);

                // pause for ten milliseconds to not be wasteful.
                // if interrupted, break.
                try { Thread.sleep(10); } catch (InterruptedException e) { break; }
            }
            shooterKickForwardMotor.set(1);
        }).thenAccept((any) -> {
            this._running = false;
        });

    }

}
