package frc.robot.autonomous;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.swerveCode.Drivetrain;

public class MovementUtil {

    public static final double acceptableErrorTranslationMeters = 0.1;
    public static final double acceptableErrorRotationDegrees = 3;

    public final Drivetrain m_swerve;


    public MovementUtil(Drivetrain drivetrai) {
        m_swerve = drivetrai;
    }

    
    /**
     * Range of error: {@value #acceptableErrorTranslationMeters} 
     * <p>Change at {@link #acceptableErrorTranslationMeters}</p>
     * @param target Pose on field.
     * @return Whether or not goal is reached.
     */
    protected boolean reachedTranslationTarget(Pose2d target) {
        return this.reachedTranslationTarget(target.getTranslation());
    }

    /**
     * Range of error: {@value #acceptableErrorTranslationMeters} 
     * <p>Change at {@link #acceptableErrorTranslationMeters}</p>
     * @param target Translation goal.
     * @return Whether or not goal is reached.
     */
    protected boolean reachedTranslationTarget(Translation2d target) {
        m_swerve.updateOdometry();
        Pose2d current = m_swerve.m_odometry.getPoseMeters();
        Translation2d delta = target.minus(current.getTranslation());
        double transDist = Math.sqrt(Math.pow(delta.getX(), 2) + Math.pow(delta.getY(), 2));
        return transDist < acceptableErrorTranslationMeters;
    }

    /**
     * Range of error: {@value #acceptableErrorRotationDegrees} 
     * <p>Change at {@link #acceptableErrorRotationDegrees}</p>
     * @param target Pose on field.
     * @return Whether or not goal is reached.
     */
    protected boolean reachedRotationTarget(Pose2d target) {
        return this.reachedRotationTarget(target.getRotation());
    }

    /**
     * Range of error: {@value #acceptableErrorRotationDegrees} 
     * <p>Change at {@link #acceptableErrorRotationDegrees}</p>
     * @param target Rotation target.
     * @return Whether or not goal is reached.
     */
    protected boolean reachedRotationTarget(Rotation2d target) {
        m_swerve.updateOdometry();
        Pose2d current = m_swerve.m_odometry.getPoseMeters();
        double rotDist = Math.abs(target.getDegrees() - current.getRotation().getDegrees());
        return rotDist < acceptableErrorRotationDegrees;
    }


    /**
     * Ranges of error:
     *  <p>Translation error range: {@value #acceptableErrorTranslationMeters} (click to change) </p>
     *  <p>Rotation error range:    {@value #acceptableErrorRotationDegrees} (click to change) </p>
     * @param target Translation goal.
     * @return Whether or not goal is reached.
     */
    protected boolean reachedFullTarget(Pose2d target) {
        return this.reachedRotationTarget(target) && this.reachedTranslationTarget(target);
    }

}