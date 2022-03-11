package frc.robot.pneumatics.basePneumatics;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class CustomDoubleSolenoid implements ISolenoid {

    private final DoubleSolenoid doublePCM;
    // private final doublePH;

    public CustomDoubleSolenoid(int forwardPort, int reversePort) {
        this.doublePCM = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, forwardPort, reversePort);
    }

    @Override
    public Value getState() {
        return this.doublePCM.get();
    }

    @Override
    public void extendFully() {
        if (this.doublePCM.get() == kForward) {
            // this.relax();
            return;
        }
        this.doublePCM.set(kForward);
    }

    @Override
    public void retractFully() {
        if (this.doublePCM.get() == kReverse) {
            // this.relax();
            return;
        }
        this.doublePCM.set(kReverse);
    }

    /**
     * @return boolean
     */
    @Override
    public boolean toggle() {
        // does nothing if off.
        if (this.doublePCM.get() == kOff)
            return false;
        this.doublePCM.toggle();
        return true;
    }

    @Override
    public void relax() {
        this.doublePCM.set(kOff);
    }

}