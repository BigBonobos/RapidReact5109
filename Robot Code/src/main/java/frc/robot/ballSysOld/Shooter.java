package frc.robot.ballSysOld;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.BangBangController;

public class Shooter {
    private CANSparkMax shooterFlyWheel;
    private CANSparkMax shooterHood;
    private RelativeEncoder shooterEncoder;
    private BangBangController overSpeedController;
    private int shooterRpms;

    public enum ShooterState {
        kRunning,
        kCoasting
    }

    public ShooterState state;

    public Shooter(int flyWheelPort, int hoodPort, int shooterSpeedRpms) {
        shooterFlyWheel = new CANSparkMax(flyWheelPort, MotorType.kBrushed);
        shooterFlyWheel.setIdleMode(CANSparkMax.IdleMode.kCoast);
        shooterHood = new CANSparkMax(hoodPort, MotorType.kBrushed);
        overSpeedController = new BangBangController();
        shooterRpms = shooterSpeedRpms;
        state = ShooterState.kRunning;
    }

    public void setShooter() {
        switch (state) {
            case kRunning:
                shooterFlyWheel.set(overSpeedController.calculate(shooterEncoder.getVelocity(), shooterRpms));
                break;
            case kCoasting:
                shooterFlyWheel.set(0);
                break;
        }
    }

}
