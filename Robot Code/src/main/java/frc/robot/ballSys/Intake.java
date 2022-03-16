package frc.robot.ballSys;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.controllers.BaseController;
public class Intake implements BaseController {
    /**
     * The motor is controlled via rotation direction.
     * Values ABOVE zero = motor is spinning inward, taking balls in.
     * Values BELOW zero = motor is spinning outward, ejecting balls.
     */
    private final CANSparkMax intakeMotor;

    public int ballCount;

    /**
     * @param motorPort
     */
    public Intake(int motorPort) {
        intakeMotor = new CANSparkMax(motorPort, MotorType.kBrushless);
    }

    public boolean isMotorActive() {
        return this.intakeMotor.get() != 0;
    }

    public boolean isMotorIntaking() {
        return this.intakeMotor.get() > 0;
    }

    public boolean isMotorEjecting() {
        return this.intakeMotor.get() < 0;
    }

    /**
     * Turns on the motor if the ball count is NOT zero.
     * If it is, don't do anything.
     */
    public void ejectBalls() {
        this.intakeMotor.set(ballCount == 0 ? 0 : -0.5);
    }

    /**
     * Turn on motor to intake if ball count is NOT two.
     * If it is, don't do anything.
     */
    public void intakeBalls() {
        this.intakeMotor.set(ballCount == 2 ?  0 : 1);
     
    }

    /**
     * Slightly dumb. It does not update ball count for the duration of intaking balls.
     * <p> Solution: move DigitalObjects into here. Issue, I am not sure if that breaks anything.
     * <p>Resets system after completing.
     * @param seconds # of seconds intake will be on.
     */
    public void intakeFor(double seconds) {
        double start = Timer.getFPGATimestamp();
        while (Timer.getFPGATimestamp() - start < seconds) {
            intakeBalls();

            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                break;
            }
        }

        resetSystem();
    }

    /**
     * Listens to RIGHT BUMPER and LEFT BUMPER.
     */
    @Override
    public void handleInputs(XboxController xController, Joystick j_operator) {
        if (xController.getRightBumper()) {
            intakeBalls();
        } else if (xController.getLeftBumper()) {
            ejectBalls();
        } else {
            resetSystem();
        }

    }

    @Override
    public void resetSystem() {
        this.intakeMotor.set(0);

    }

    // public void intakeBalls() {
    // if (this.canIntakeBalls()) {
    // this.intakeMotor.set(0.3);
    // return;
    // }
    // this.intakeSolenoid.extendFully();
    // this.intakeMotor.set(0.3);
    // }

}

// public class Intake {

// /**
// * The motor is controlled via rotation direction.
// * Values ABOVE zero = motor is spinning inward, taking balls in.
// * Values BELOW zero = motor is spinning outward, ejecting balls.
// */
// private final CANSparkMax intakeMotor;
// private final SSolenoid intakeSolenoid;

// /**
// *
// * @param motorPort
// * @param solenoidPort
// */
// public Intake(int motorPort, int solenoidPort) {
// intakeMotor = new CANSparkMax(motorPort, MotorType.kBrushless);
// intakeSolenoid = new SSolenoid(solenoidPort);

// }

// public boolean isExtended() {
// return this.intakeSolenoid.getState() == Value.kForward;
// }

// public boolean isMotorActive() {
// return this.intakeMotor.get() != 0;
// }

// public boolean isMotorIntaking() {
// return this.intakeMotor.get() > 0;
// }

// public boolean isMotorEjecting() {
// return this.intakeMotor.get() < 0;
// }

// /**
// * For now, this method is redundant. Extend this method if new changes.
// *
// * @return Whether or not the intake can take balls in.
// */
// public boolean canIntakeBalls() {
// return this.isExtended(); // && this.isMotorIntaking();
// }

// /**
// * For now, this method is redundant. Extend this method if new changes.
// *
// * @return Whether or not the intake can eject balls.
// */
// public boolean canEjectBalls() {
// return this.isExtended(); // && this.isMotorEjecting();
// }

// /**
// * We do not have a method to detect whether or not balls are in the intake
// currently.
// * We probably will not, either.
// */
// public void ejectBalls() {
// if (this.canEjectBalls()) {
// this.intakeMotor.set(-1);
// return;
// }
// this.intakeSolenoid.extendFully();
// this.intakeMotor.set(-1);
// }

// /**
// * We do not have a method to detect whether or not balls are in the intake
// currently.
// * We probably will not, either.
// */
// public void intakeBalls() {
// if (this.canIntakeBalls()) {
// this.intakeMotor.set(0.3);
// return;
// }
// this.intakeSolenoid.extendFully();
// this.intakeMotor.set(0.3);
// }

// public void relax() {
// this.intakeMotor.set(0);
// }

// /**
// * lol, good name.
// */
// public void closeUpShop() {
// this.relax();
// this.intakeSolenoid.retractFully();
// }

// // Custom methods, requested by Xander.

// /**
// * Opens and closes the intake gate, depending on boolean input.
// *
// * @param open
// * true = forward, open. false = reverse, closed.
// */
// public void moveIntakeGate(boolean open) {
// if (open)
// this.intakeSolenoid.extendFully();
// else
// this.intakeSolenoid.retractFully();
// }

// /**
// * Turns the intake motor a certain direction, depending on boolean input.
// *
// * @param inward
// * true = inward, backward. false = outward, forward.
// */
// public void setIntakeMotorMode(boolean inward) {
// if (inward)
// this.intakeMotor.set(1);
// else
// this.intakeMotor.set(-1);
// }
// }