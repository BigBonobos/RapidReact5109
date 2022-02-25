package frc.robot.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.lib.pneumatics.basePnuematics.SingleSolenoid;

public class Intake {

    /**
     * The motor is controlled via rotation direction.
     * Values ABOVE zero = motor is spinning inward, taking balls in.
     * Values BELOW zero = motor is spinning outward, ejecting balls.
     */
    private final CANSparkMax intakeMotor;
    private final SingleSolenoid intakeSolenoid;

    public Intake(int motorPort, int solenoidPort) {
        intakeMotor = new CANSparkMax(motorPort, MotorType.kBrushless);
        intakeSolenoid = new SingleSolenoid(solenoidPort);

    }

    public boolean isExtended() {
        return this.intakeSolenoid.getState() == Value.kForward;
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
     * For now, this method is redundant. Extend this method if new changes.
     * 
     * @return Whether or not the intake can take balls in.
     */
    public boolean canIntakeBalls() {
        return this.isExtended(); // && this.isMotorIntaking();
    }

    /**
     * For now, this method is redundant. Extend this method if new changes.
     * 
     * @return Whether or not the intake can eject balls.
     */
    public boolean canEjectBalls() {
        return this.isExtended(); // && this.isMotorEjecting();
    }

    /**
     * We do not have a method to detect whether or not balls are in the intake currently.
     * We probably will not, either.
     */
    public void ejectBalls() {
        if (this.canEjectBalls()) {
            this.intakeMotor.set(-1);
            return;
        }
        this.intakeSolenoid.extendFully();
        this.intakeMotor.set(-1);
    }

    /**
     * We do not have a method to detect whether or not balls are in the intake currently.
     * We probably will not, either.
     */
    public void intakeBalls() {
        if (this.canIntakeBalls()) {
            this.intakeMotor.set(0.3);
            return;
        }
        this.intakeSolenoid.extendFully();
        this.intakeMotor.set(0.3);
    }

    public void relax() {
        this.intakeMotor.set(0);
    }

    /**
     * lol, good name.
     */
    public void closeUpShop() {
        this.relax();
        this.intakeSolenoid.retractFully();
    }

    // Custom methods, requested by Xander.

    /**
     * Opens and closes the intake gate, depending on boolean input.
     * 
     * @param open
     *            true = forward, open. false = reverse, closed.
     */
    public void moveIntakeGate(boolean open) {
        if (open)
            this.intakeSolenoid.extendFully();
        else
            this.intakeSolenoid.retractFully();
    }

    /**
     * Turns the intake motor a certain direction, depending on boolean input.
     * 
     * @param inward
     *            true = inward, backward. false = outward, forward.
     */
    public void setIntakeMotorMode(boolean inward) {
        if (inward)
            this.intakeMotor.set(1);
        else
            this.intakeMotor.set(-1);
    }
}
