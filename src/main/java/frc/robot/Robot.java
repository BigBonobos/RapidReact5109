// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// wheas da fwisbee

package frc.robot;

import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public DigitalInput line1 = new DigitalInput(0);
  public DigitalInput line2 = new DigitalInput(1);
  public CANSparkMax motor = new CANSparkMax(1, MotorType.kBrushless); 
  boolean hasBall = false;

  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {

    // HasBall covers edge case of ball getting stuck between the two sensors thus making the sensors read FF (false false)
    // Cases TT, FT
    if ((line1.get() == true && line2.get() == true) || (line1.get() == false && line2.get() == true)) {
        motor.stopMotor();
        hasBall = false;
    }

    // Cases TF
    if (line1.get() == true && line2.get() == false) {

      // arbitrary motor value
      motor.set(0.5);
      hasBall = true;
    }

    // Case FF + HB (ball stuck)
    if (line1.get() == false && line2.get() == false && hasBall == true) {
      motor.set(0.5);

    // Case FF (ball not stuck)
    } else {
      motor.stopMotor();
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}
