package frc.robot.controllers;

import frc.robot.pneumatics.base.ISolenoid;

public interface SolenoidController extends BaseController {
    
    public abstract void extendSolenoids();


    public abstract void retractSolenoids();


    public abstract void toggleSolenoids();

    public abstract ISolenoid[] getAllSolenoids();

}
