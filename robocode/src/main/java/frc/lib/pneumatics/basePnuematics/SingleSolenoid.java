package frc.lib.pneumatics.basePnuematics;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class SingleSolenoid implements ISolenoid {

    private final Solenoid singlePCM;

    public SingleSolenoid(int port) {
        this.singlePCM = new Solenoid(PneumaticsModuleType.CTREPCM, port);

    }

    @Override
    public Value getState() {
        return this.singlePCM.get() ? Value.kForward : Value.kOff;
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
