// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerveCode;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.ctre.phoenix.sensors.*;

public class SwerveModule implements RevOptimization {
  private static final double kWheelRadius = 0.0508;

  private static final double kTicksPerMotorRadian = 42 / (2 * Math.PI);
  private static final double kTicksPerWheelRadian = kTicksPerMotorRadian * 8.14;

  private static final double kTicksPerTurnerWheelRadian = 42 / (2 * Math.PI) * 12.8;

  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

  private final CANSparkMax m_driveMotor;
  public final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turningEncoderRelative;
  public final CANCoder m_turningEncoderAbsolute;

  private final SparkMaxPIDController m_drivePIDController;
  private final SparkMaxPIDController m_turningPIDController;

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel   ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModule(int driveMotorChannel, int turningMotorChannel, int canCoderId, double magOffset) {

    // Motor Instantiation
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

    m_driveMotor.setIdleMode(IdleMode.kCoast);
    m_turningMotor.setIdleMode(IdleMode.kCoast);

    // Encoder Instantiation
    m_driveEncoder = m_driveMotor.getEncoder();
    m_turningEncoderRelative = m_turningMotor.getEncoder();
    m_turningEncoderAbsolute = new CANCoder(canCoderId);

    // PID Instantiation
    m_drivePIDController = m_driveMotor.getPIDController();
    m_turningPIDController = m_turningMotor.getPIDController();

    // Setting PID Values
    m_drivePIDController.setP(0.3);
    m_drivePIDController.setI(0);
    m_drivePIDController.setD(0);

    m_turningPIDController.setP(1.5);
    m_turningPIDController.setI(0);
    m_turningPIDController.setD(0);
    // m_turningPIDController.setIMaxAccum(0.1,0);

    // Setting turn constraints
    m_turningPIDController.setSmartMotionAccelStrategy(com.revrobotics.SparkMaxPIDController.AccelStrategy.kTrapezoidal,
        0);
    m_turningPIDController.setSmartMotionMaxAccel(kModuleMaxAngularAcceleration, 0);
    m_turningPIDController.setSmartMotionMaxVelocity(kModuleMaxAngularVelocity, 0);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    // m_driveEncoder.setPositionConversionFactor(2 * Math.PI * kWheelRadius /
    // kEncoderResolution);

    m_driveEncoder.setVelocityConversionFactor(1 / (kTicksPerWheelRadian) * kWheelRadius);
    m_turningEncoderRelative.setPositionConversionFactor(1 / kTicksPerTurnerWheelRadian);

    // m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * wpi::math::pi)
    // divided by the encoder resolution.
    CANCoderConfiguration encoderConfig = new CANCoderConfiguration();
    encoderConfig.magnetOffsetDegrees = magOffset;
    encoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    encoderConfig.unitString = "deg";
    encoderConfig.sensorDirection = false; // Counter clock-wise
    encoderConfig.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
    m_turningEncoderAbsolute.configAllSettings(encoderConfig);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    // m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
      m_turningPIDController.setOutputRange(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveEncoder.getVelocity(),
        new Rotation2d(m_turningEncoderAbsolute.getAbsolutePosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param state Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState state) {

    // final double driveFeedforward =
    // m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // final double turnFeedforward =
    // m_turnFeedforward.calculate(m_turningEncoder.getVelocity());

    // m_drivePIDController.setFF(driveFeedforward);
    // m_turningPIDController.setFF(turnFeedforward);
    Rotation2d currentAngle = Rotation2d.fromDegrees(m_turningEncoderAbsolute.getAbsolutePosition());
    state = optimize(state, currentAngle);

    final double deltaAngle = state.angle.getRadians() - currentAngle.getRadians();

    // System.out.println("deltaAngle " + deltaAngle);
    // System.out.println("encoderPos " + m_turningEncoderRelative.getPosition());
    // System.out.println("Encoder "+ m_turningEncoderAbsolute.getPosition());

    //System.out.println("Relative:" + m_turningEncoderRelative.getPosition());

    
    double diffAngle = deltaAngle + m_turningEncoderRelative.getPosition();
  //  diffAngle = diffAngle % 2 * Math.PI;
  //   diffAngle = (diffAngle + 2 * Math.PI) % 2 * Math.PI;
  //   if (diffAngle > Math.PI) diffAngle -= 2 * Math.PI;
    double finalAngle = (diffAngle) / (2 * Math.PI);
 
    

    // System.out.pr  intln("final angle: "+ diffAngle);
    // state = SwerveModuleState.optimize(state, Rotation2d.from diffAngle);
    // double result = diffAngle % 360;

    // if (result < 0) {
    // result += 360;
    // }
    //System.out.println("Target:" + diffAngle);
    // * 180 / Math.PI
    m_turningEncoderAbsolute.setPositionToAbsolute();
    m_drivePIDController.setReference(state.speedMetersPerSecond / kWheelRadius, ControlType.kSmartVelocity);
    m_turningPIDController.setReference(finalAngle, ControlType.kSmartMotion);
  }
}
