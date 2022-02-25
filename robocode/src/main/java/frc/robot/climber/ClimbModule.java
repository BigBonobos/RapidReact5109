package frc.robot.climber;

import frc.lib.pneumatics.ISolenoidController;
import frc.lib.pneumatics.basePnuematics.CustomDoubleSolenoid;
import frc.lib.pneumatics.basePnuematics.ISolenoid;

public class ClimbModule implements ISolenoidController {

    private final CustomDoubleSolenoid rightSolenoid;
    private final CustomDoubleSolenoid leftSolenoid;

    public ClimbModule(int[][] solenoidSettings) {
        if (solenoidSettings.length != 2)
            throw new IllegalArgumentException("You fucked up. Only two solenoids for this.");
        for (int i = 0; i < solenoidSettings.length; i++) {
            if (solenoidSettings[i].length != 2)
                throw new IllegalArgumentException(
                        String.format(
                                "You fucked up. Solenoid %d (%s) has an incorrect configuration.",
                                i,
                                (i == 0 ? "right" : "left")));

        }

        rightSolenoid = new CustomDoubleSolenoid(solenoidSettings[0][0], solenoidSettings[0][1]);
        leftSolenoid = new CustomDoubleSolenoid(solenoidSettings[1][0], solenoidSettings[1][1]);

    }

    public ClimbModule(CustomDoubleSolenoid right, CustomDoubleSolenoid left) {
        rightSolenoid = right;
        leftSolenoid = left;
    }

    @Override
    public void extendAll() {
        rightSolenoid.extendFully();
        leftSolenoid.extendFully();

    }

    @Override
    public void retractAll() {
        rightSolenoid.retractFully();
        leftSolenoid.retractFully();

    }

    @Override
    public void toggleAll() {
        rightSolenoid.toggle();
        leftSolenoid.toggle();

    }

    @Override
    public ISolenoid[] getAllSolenoids() {
        ISolenoid[] sols = { rightSolenoid, leftSolenoid };
        return sols;
    }

}
