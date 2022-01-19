// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
  private static final double kWheelRadius = 0.0508;
  private static final double kEncoderResolution = 42 * 8.14;

  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration =
      2 * Math.PI; // radians per second squared

  private final CANSparkMax m_drive;
  private final CANSparkMax m_turning;

  private final RelativeEncoder e_drive;
  private final RelativeEncoder e_turning;

  private final SparkMaxPIDController m_drivePIDController;
  private final SparkMaxPIDController m_turningPIDController;

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModule(int driveMotorChannel, int turningMotorChannel) {

    // Motor Instantiation
    m_drive = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turning = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

    // Encoder Instantiation
    e_drive = m_drive.getEncoder();
    e_turning = m_turning.getEncoder();

    // PID Instantiation
    m_drivePIDController = m_drive.getPIDController();
    m_turningPIDController = m_turning.getPIDController();
    
    // Setting PID Values
    m_drivePIDController.setP(1);
    m_drivePIDController.setI(0);
    m_drivePIDController.setD(0);

    m_turningPIDController.setP(1);
    m_turningPIDController.setI(0);
    m_turningPIDController.setD(0);
    

    // Setting turn constraints
    m_turningPIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
    m_turningPIDController.setSmartMotionMaxAccel(kModuleMaxAngularAcceleration, 0);
    m_turningPIDController.setSmartMotionMaxVelocity(kModuleMaxAngularVelocity, 0);
    

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    e_drive.setPositionConversionFactor(2 * Math.PI * kWheelRadius / kEncoderResolution);

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * wpi::math::pi)
    // divided by the encoder resolution.
    e_turning.setPositionConversionFactor(2 * Math.PI / kEncoderResolution);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    // m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(e_drive.getVelocity(), new Rotation2d(e_turning.getPosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param state Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState state) {

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    final double turnFeedforward =
        m_turnFeedforward.calculate(e_turning.getVelocity());

    m_drivePIDController.setFF(driveFeedforward);
    m_turningPIDController.setFF(turnFeedforward);
  }
}
