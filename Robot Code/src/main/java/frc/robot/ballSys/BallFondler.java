package frc.robot.ballSys;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.controllers.BaseController;

public class BallFondler implements BaseController {

    public final Intake intake;
    public final Shooter shooter;

    public final DigitalInput beam1 = new DigitalInput(0); // Outside the kicker wheel
    public final DigitalInput beam2 = new DigitalInput(1); // Inside the kicker wheel

    public int ballCount;
    public boolean shooting;
    public boolean intaking;

    public BallFondler(int m_intakePort, int m_indexPort, int m_shooterPort) {
        intake = new Intake(m_intakePort);
        shooter = new Shooter(m_indexPort, m_shooterPort, 3500);

    }

    public int getBallCount() {
        if (!beam1.get()) {
            ballCount = 2;
        }
        if (!beam2.get() && beam1.get()) {
            ballCount = 1;
        }
        if (beam1.get() && beam2.get()) {
            ballCount = 0;
        }

        return ballCount;
    }

    @Override
    public void handleInputs(XboxController xController, Joystick j_operator) {

        // update in here instead of separate method to ensure updated values.
        getBallCount();
        intake.ballCount = ballCount;
        shooter.ballCount = ballCount;

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
