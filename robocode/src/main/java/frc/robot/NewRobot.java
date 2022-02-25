package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.climber.ClimbModule;
import frc.robot.shooter.Intake;

public class NewRobot extends TimedRobot {

    // public CANSparkMax motor1 = new CANSparkMax(14,MotorType.kBrushless);

    /**
     * 
     * Currently unneeded.
     */
    // private int listenerHandleBall;
    // private int listenerHandleShooter;
    // private boolean intakeRunning;
    // private boolean autoAlignRunningBall;
    // private boolean autoAlignRunningShooter;

    /**
     * Variable assignment for limelight.
     * Currently unnused.
     */
    // private boolean autoAlignRunningShooter = false;
    // private boolean autoAlignRunningBall = false;
    private double autoAlignRange = 360.0;
    // private int autoCounter = 0;

    /**
     * Settings for drivetrainModule.
     * The settings are: @see {@link frc.lib.swerveCode.SwerveModule#SwerveModule(int, int, int, double)}
     */
    private static final double[] 
            frontLeftMotorIDs = { 15, 14, 1, 162.6 },
            frontRightMotorIDs = { 12, 13, 2, -163.213 },
            backLeftMotorIDs = { 18, 19, 3, 60.6 },
            backRightMotorIDs = { 16, 17, 4, -80.86 };// 100}; // back right?

    /**
     * Settings for climbingModule.
     * The settings are: @see {@link frc.robot.climber.ClimbModule#ClimbModule(int[][])}
     */
    private static final int[][] solenoidSettings = { { 1, 2 }, { 3, 4 } };

    /**
     * Controls the translation (x, y, z) of the bot w/o rotating it.
     */
    private final Joystick translationJoystick = new Joystick(0);

    /**
     * Controls the rotation (roll) of the bot, does not translate bot.
     */
    private final Joystick rotationJoystick = new Joystick(1);

    /**
     * Used to control other aspects of the bot (modules).
     */
    private final Joystick OperatorJoystick = new Joystick(2);

    /**
     * A custom implementation of four SwerveModules (also custom).
     * Can omni-directionally drive.
     * Control via @see {@link frc.robot.NewRobot#driveWithJoystick(boolean)};
     */
    private final Drivetrain drivetrainModule = new Drivetrain(
            autoAlignRange, frontLeftMotorIDs, frontRightMotorIDs, backLeftMotorIDs, backRightMotorIDs);

    /**
     * Custom intake module. Takes one motor and one solenoid.
     * The settings are: @see {@link frc.robot.shooter.Intake#Intake(int, int)}
     */
    private final Intake intakeModule = new Intake(9, 0);

    /**
     * custom climb module. Two double solenoids control the arms to pull the bot upward.
     * Referenced earlier, but here: @see {@link frc.robot.climber.ClimbModule#ClimbModule(int[][])}
     */
    private final ClimbModule climbModule = new ClimbModule(solenoidSettings);

    /**
     * SlewRateLimiters limit the ROC of an inputs strength.
     * <p>For example, cannot instantly go from rest -> full speed.</p>
     * <p>Technical term is "interpolating" inbetween points until reaching full speed. </p>
     * Basically, makes controls more gradual.
     * Lower = jerkier. Higher = smoother but slower.
     * <p>setting of 10: 1/3 sec from 0 to 1.</p>
     */
    private final SlewRateLimiter 
            JoystickSpeedLimiterX = new SlewRateLimiter(10),
            JoystickSpeedLimiterY = new SlewRateLimiter(10),
            JoystickSpeedLimiterRot = new SlewRateLimiter(10);

    /**
     * Test stub. Called once upon initialization.
     */
    public void testInit() {
        drivetrainModule.customAutoAlign();
    }

    /**
     * Test stub. This is a continual loop. AFAIK, there is no pause between iterations.
     */
    @Override
    public void testPeriodic() {
        driveWithJoystick(false);

    }

    /**
     * Init when setting up teleop setting.
     */
    @Override
    public void teleopPeriodic() {
        driveWithJoystick(true);
    }

    /**
     * 
     * @param fieldRelative
     *            whether or not the offset should be based off of pos/rot of bot, 
     *            or pos/rot of field (which doesn't change).
     */
    private void driveWithJoystick(final boolean fieldRelative) {
        /**
         * Get desired X speed of chassis.
         * Inverted since Xbox joysticks return flipped values.
         * 
         * <p>
         * UPDATE: We're wrong. Xbox is correct. The bot drives backwards.
         * </p>
         */
        final double xSpeed = -JoystickSpeedLimiterX.calculate(MathUtil.applyDeadband(translationJoystick.getY(), 0.08))
                * frc.robot.Drivetrain.kMaxSpeed;

        /**
         * Get desired Y (strafe/sideways) speed of chassis.
         * Positive = left, negative = right.
         * XBox controllers return flipped values.
         * 
         * <p>
         * UPDATE: We're wrong. Xbox is correct. The bot drives backwards.
         * </p>
         */
        final double ySpeed = -JoystickSpeedLimiterY.calculate(MathUtil.applyDeadband(translationJoystick.getX(), 0.08))
                * frc.robot.Drivetrain.kMaxSpeed;

        /**
         * Get desired rotation speed of chassis.
         * Positive = left, negative = right.
         * Xbox returns positive when holding right by default.
         * 
         * <p>
         * UPDATE: We're wrong. Xbox is correct. The bot drives backwards.
         * </p>
         */
        final double rot = -JoystickSpeedLimiterRot.calculate(MathUtil.applyDeadband(rotationJoystick.getY(), 0.08))
                * frc.robot.Drivetrain.kMaxAngularSpeed;

        /**
         * Call internal drive method of drivetrainModule.
         */
        drivetrainModule.drive(xSpeed, ySpeed, rot, fieldRelative);
    }

    private void handleControllerInputs() {

    }

}
