package frc.robot.ballSys;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.BangBangController;

public class Shooter {
    private CANSparkMax m_shooterFlyWheel;
    private CANSparkMax m_shooterHood;
    private SparkMaxPIDController pc_shooterHood;
    private RelativeEncoder e_shooterFlywheel;
    private BangBangController overSpeedController;
    private int shooterRpms;

    public static enum ShooterState {
        kRunning,
        kCoasting
    }

    private static enum HoodState {
        Raised,
        Lowered
    }

    public ShooterState state;
    private HoodState hoodState;

    public Shooter(int flyWheelPort, int hoodPort, int shooterSpeedRpms) {
        m_shooterFlyWheel = new CANSparkMax(flyWheelPort, MotorType.kBrushed);
        m_shooterHood = new CANSparkMax(hoodPort, MotorType.kBrushed);

        m_shooterFlyWheel.setIdleMode(IdleMode.kCoast);
        m_shooterHood.setIdleMode(IdleMode.kCoast);

        overSpeedController = new BangBangController();
        shooterRpms = shooterSpeedRpms;
        state = ShooterState.kRunning;

        pc_shooterHood = m_shooterHood.getPIDController();
        pc_shooterHood.setP(.1);
        pc_shooterHood.setI(0);
        pc_shooterHood.setD(0);
    }

    public void setShooter() {
        switch (state) {
            case kRunning:
                raiseHood();
                m_shooterFlyWheel.set(overSpeedController.calculate(e_shooterFlywheel.getVelocity(), shooterRpms));
                break;
            case kCoasting:
                lowerHood();
                m_shooterFlyWheel.set(0);
                break;
        }
    }

    public void raiseHood() {
        if (hoodState == HoodState.Lowered) {
            pc_shooterHood.setReference(20, ControlType.kVelocity);
            hoodState = HoodState.Raised;
        }
    }

    public void lowerHood() {
        if (hoodState == HoodState.Raised) {
            m_shooterHood.set(0);
            hoodState = HoodState.Lowered;
        }
    }
}
