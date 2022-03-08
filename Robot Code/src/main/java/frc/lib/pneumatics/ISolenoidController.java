package frc.lib.pneumatics;

import frc.lib.pneumatics.basePnuematics.ISolenoid;

public interface ISolenoidController {
    
    public abstract void extendAll();


    public abstract void retractAll();


    public abstract void toggleAll();

    public abstract ISolenoid[] getAllSolenoids();

}
