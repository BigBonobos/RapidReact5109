package frc.robot.climb;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.controllers.SolenoidController;
import frc.robot.pneumatics.base.CDSolenoid;
import frc.robot.pneumatics.base.ISolenoid;

public class ClimbModule implements SolenoidController {

    private final CDSolenoid rightSolenoid;
    private final CDSolenoid leftSolenoid;
    private final ISolenoid[] sols;

    private final CANSparkMax climbMotor;

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
        rightSolenoid = new CDSolenoid(solenoidSettings[0][0], solenoidSettings[0][1]);
        leftSolenoid = new CDSolenoid(solenoidSettings[1][0], solenoidSettings[1][1]);
        sols = new ISolenoid[]{ rightSolenoid, leftSolenoid };

    }

    public ClimbModule(CANSparkMax climb, CDSolenoid right, CDSolenoid left) {
        climbMotor = climb;
        rightSolenoid = right;
        leftSolenoid = left;
        sols = new ISolenoid[]{ left, right };
    }

    @Override
    public void extendSolenoids() {
        rightSolenoid.extendFully();
        leftSolenoid.extendFully();

    }

    @Override
    public void retractSolenoids() {
        rightSolenoid.retractFully();
        leftSolenoid.retractFully();

    }

    @Override
    public void toggleSolenoids() {
        rightSolenoid.toggle();
        leftSolenoid.toggle();

    }

    @Override
    public ISolenoid[] getAllSolenoids() {
        return sols;
    }


    @Override
    public void handleInputs(XboxController xController, Joystick j_operator) {
        if (xController.getBButton()) {
            climbMotor.set(-0.5);
          } else if (xController.getAButton()) {
            climbMotor.set(0.5);
          } else {
            climbMotor.set(0);
          }
    }

    @Override
    public void resetSystem() {
        retractSolenoids();
        climbMotor.set(0);
    }

}
