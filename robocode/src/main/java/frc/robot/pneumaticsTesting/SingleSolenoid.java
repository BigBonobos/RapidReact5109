package frc.robot.pneumaticsTesting;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class SingleSolenoid extends ISolenoid {

    private final Solenoid singlePCM;

    public SingleSolenoid(int forwardPort, int reversePort) {
        super(forwardPort, reversePort);

        this.singlePCM = new Solenoid(PneumaticsModuleType.CTREPCM, 1);

    }

    @Override
    public void extendFully() {
        if (this.singlePCM.get())
            return;
        this.singlePCM.set(true);

    }

    @Override
    public void retractFully() {
        if (!this.singlePCM.get())
            return;
        this.singlePCM.set(false);

    }

    /**
     * @return boolean
     */
    @Override
    public boolean toggle() {
        // there is no default value, so always return true.
        this.singlePCM.toggle();
        return true;
    }

    @Override
    public void relax() {}

}
