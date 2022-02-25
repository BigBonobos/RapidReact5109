package frc.lib.pneumatics.basePnuematics;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public interface ISolenoid {
    
    public abstract Value getState();

    public abstract void extendFully();

    public abstract void retractFully();

    public abstract boolean toggle();

    public abstract void relax();


}
