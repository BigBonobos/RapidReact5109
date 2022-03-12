package frc.robot.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

public interface BaseController {

    public void handleInputs(XboxController xController, Joystick j_operator);


    public void resetSystem();
}
