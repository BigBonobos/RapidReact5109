package frc.robot.climb;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.controllers.SolenoidController;
import frc.robot.pneumatics.base.SSolenoid;

public class ClimbModule implements SolenoidController {

    private final SSolenoid bothSolenoids;
    // private final SSolenoid leftSolenoid;
    private final SSolenoid[] sols;

    private final CANSparkMax climbMotor;
    private final CANSparkMax hookMotor = new CANSparkMax(11, MotorType.kBrushless);

    public ClimbModule(int motorPort, int[][] solenoidSettings) {
        if (solenoidSettings.length != 2)
            throw new IllegalArgumentException("You fucked up. Only two solenoids for this.");
        for (int i = 0; i < solenoidSettings.length; i++) {
            if (solenoidSettings[i].length != 2)
                throw new IllegalArgumentException(
                        String.format(
                                "You fucked up. Solenoid %i (%s) has an incorrect configuration.",
                                i,
                                (i == 0 ? "right" : "left")));

        }

        climbMotor = new CANSparkMax(motorPort, MotorType.kBrushless);
        bothSolenoids = new SSolenoid(solenoidSettings[0][0]);
        // leftSolenoid = new SSolenoid(solenoidSettings[1][0], solenoidSettings[1][1]);
        sols = new SSolenoid[]{ bothSolenoids };

    }

    public ClimbModule(CANSparkMax climb, SSolenoid both) {
        climbMotor = climb;
        bothSolenoids = both;
        sols = new SSolenoid[]{ both };
    }

    @Override
    public void extendSolenoids() {
        bothSolenoids.extendFully();
        // leftSolenoid.extendFully();

    }

    @Override
    public void retractSolenoids() {
        bothSolenoids.retractFully();
        // leftSolenoid.retractFully();

    }

    @Override
    public void toggleSolenoids() {
        bothSolenoids.toggle();
        // leftSolenoid.toggle();

    }

    @Override
    public SSolenoid[] getAllSolenoids() {
        return sols;
    }


    /**
     * Listens to Joystick Input from Operator
     */
    @Override
    public void handleInputs(XboxController xController, Joystick j_operator) {
        if (j_operator.getRawButton(4)) {
            climbMotor.set(-0.5);
        } else if (j_operator.getRawButton(5)) {
            climbMotor.set(0.5);
        } else {
            climbMotor.set(0);
        }

        if (j_operator.getTrigger()) {
            extendSolenoids();
        } else if (j_operator.getRawButton(3)) {
            retractSolenoids();
        }

        if (j_operator.getRawButton(2)) {
            hookMotor.set(.1);
        } else if (j_operator.getRawButton(6)) {
            hookMotor.set(-.1);
        } else {
            hookMotor.set(0);
        }
    }

    @Override
    public void resetSystem() {
        // retractSolenoids();
        climbMotor.set(0);
    }

}
