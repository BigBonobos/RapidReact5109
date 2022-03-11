package frc.robot.pneumatics;

import frc.robot.pneumatics.basePneumatics.ISolenoid;

public interface ISolenoidController {
    
    public abstract void extendAll();


    public abstract void retractAll();


    public abstract void toggleAll();

    public abstract ISolenoid[] getAllSolenoids();

}
