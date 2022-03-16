package frc.robot.ballSys;

import java.util.concurrent.locks.ReentrantLock;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.controllers.BaseController;

public class BallFondler implements BaseController {

    public final Intake intake;
    public final Shooter shooter;

    public final DigitalInput beam1 = new DigitalInput(0); // Outside the kicker wheel
    public final DigitalInput beam2 = new DigitalInput(1); // Inside the kicker wheel

    public final ReentrantLock lock;

    public int ballCount;
    public boolean shooting;
    public boolean intaking;

    /**
     * Wrapper around both intake and shooter. Uses DigitalInputs to update
     * ballIndexes.
     * 
     * @param m_intakePort
     * @param m_indexPort
     * @param m_shooterPort
     * @param wantedRPMs
     */
    public BallFondler(int m_intakePort, int m_indexPort, int m_shooterPort, double wantedRPMs) {
        intake = new Intake(m_intakePort);
        shooter = new Shooter(m_indexPort, m_shooterPort, wantedRPMs);
        lock = new ReentrantLock();

    }

    public int getAndUpdateBallCount() {
        if (!beam1.get()) {
            ballCount = 2;
        }
        if (!beam2.get() && beam1.get()) {
            ballCount = 1;
        }
        if (beam1.get() && beam2.get()) {
            ballCount = 0;
        }

        intake.ballCount = ballCount;
        shooter.ballCount = ballCount;
        return ballCount;
    }

    // if(shooting==false) {
    // if (ballCount == 0) {
    // if (!Beam1.get()) {
    // m_indexWheel.set(0.6);
    // }
    // if (!Beam2.get()) {
    // m_indexWheel.stopMotor();
    // ballCount = 1;
    // }
    // }
    // if (ballCount == 1) {
    // if (!Beam1.get() && !Beam2.get()) {
    // m_indexWheel.stopMotor();
    // ballCount = 2;
    // }
    // }
    // }

    /**
     * Updates ball index, then shoots the ball.
     */
    public void shoot() {
        getAndUpdateBallCount();
        shooter.newShoot();
    }

    /**
     * WIP method. Will update index so that intake stops early.
     * @param seconds
     */
    public void smartIntake(double seconds) {
        lock.lock();
        try {
            double start = Timer.getFPGATimestamp();
            while (getAndUpdateBallCount() < 2 && Timer.getFPGATimestamp() - start < seconds) {
                intake.intakeBalls();

                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    break;
                }
            }
            intake.resetSystem();
        } finally {
            lock.unlock();
        }

    }

    /**
     * Listens to LEFT BUMPER and the underlying functions.
     * 
     * @see {@link frc.robot.ballSys.Intake#handleInputs(XboxController, Joystick)}
     * @see {@link frc.robot.ballSys.Shooter#handleInputs(XboxController, Joystick)}
     */
    @Override
    public void handleInputs(XboxController xController, Joystick j_operator) {

        // update in here instead of separate method to ensure updated values.
        getAndUpdateBallCount();

        if (xController.getLeftBumper()) {
            shooter.newShoot();
        }

        intake.handleInputs(xController, j_operator);
        shooter.handleInputs(xController, j_operator);

        intaking = intake.isMotorIntaking();
        shooting = shooter.isShooting();

    }

    @Override
    public void resetSystem() {
        intake.resetSystem();
        shooter.resetSystem();

    }

    public void hardReset() {
        ballCount = 0;
        resetSystem();
    }

}
