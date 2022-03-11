package frc.robot.autonomous;

import java.util.Optional;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.swerveCode.Drivetrain;
import frc.robot.swerveCode.RevOptimization;

/**
 * Storage class for autonomous driving.
 * DO NOT USE THIS. THIS IS NOT THREAD SAFE.
 */
public abstract class Autonomous extends MovementUtil {

    public Autonomous(Drivetrain drivetrai) {
        super(drivetrai);
    }

    public boolean translateTo(Translation2d fieldPoint, Optional<Double> speed) {

        Translation2d delta = fieldPoint
                .minus(m_swerve.m_odometry.getPoseMeters().getTranslation());
        double yaw = Math.atan2(delta.getX(), delta.getY());
        double wantedSpeed = speed.isPresent() ? speed.get() : Drivetrain.kMaxAngularSpeed;

        double xSpeed = wantedSpeed * Math.sin(yaw);
        double ySpeed = wantedSpeed * Math.cos(yaw);

        while (!reachedTranslationTarget(fieldPoint)) {
            delta = fieldPoint
                    .minus(m_swerve.m_odometry.getPoseMeters().getTranslation());
            yaw = Math.atan2(delta.getX(), delta.getY());

            xSpeed = wantedSpeed * Math.sin(yaw);
            ySpeed = wantedSpeed * Math.cos(yaw);
            m_swerve.drive(xSpeed, ySpeed, 0, false);
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                return false;
            }

        }
        return true;
    }

    public boolean rotateTo(Rotation2d absRotation, Optional<Double> rotSpeed) {
        double current = m_swerve.navX.getAngle();
        double wanted = RevOptimization.placeInAppropriate0To360Scope(current, absRotation.getDegrees());
        double wantedSpeed = rotSpeed.isPresent() ? rotSpeed.get() : Drivetrain.kMaxAngularSpeed;

        while (!reachedRotationTarget(absRotation)) {

            // I may be using this incorrectly. :thumbsup:
            ChassisSpeeds test = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, wantedSpeed,
                    Rotation2d.fromDegrees(wanted));
            m_swerve.drive(test.vxMetersPerSecond, test.vyMetersPerSecond, test.omegaRadiansPerSecond, true);
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                return false;
            }
        }

        return true;

    }

    /**
     * Wasteful variable assignments are for code readability. I'm sorry, Rob. I
     * cannot optimize this.
     * <p>
     * Also, this does translation, then rotation. It does not run it together.
     * </p>
     * I can fix that later.
     * 
     * @param fieldPose
     *                  Wanted position on the field. Both translation and rotation.
     * @param speed
     *                  Optional speed wanted. Defaults to max.
     */
    public void translateToFieldPoint(Pose2d fieldPose, Optional<Double> speed, Optional<Double> rotSpeed) {
        while (!reachedFullTarget(fieldPose)) {
            if (!reachedTranslationTarget(fieldPose)) {
                translateTo(fieldPose.getTranslation(), speed);
            }

            if (reachedTranslationTarget(fieldPose) && !reachedRotationTarget(fieldPose)) {
                rotateTo(fieldPose.getRotation(), rotSpeed);
            }
        }

    }

    public enum DriveDirection {
        FORWARD(0),
        BACKWARD(180),
        RIGHT(270),
        LEFT(90);

        private final int rotation;

        private int getRot() {
            return rotation;
        };

        private DriveDirection( int rotation) {
            this.rotation = rotation;
        }
    }

    public boolean driveTowards(DriveDirection dir, double distance, Optional<Double> speed) {
        return this.driveTowards(dir.getRot(), distance, speed);
    }

    public boolean driveTowards(double angle, double distance, Optional<Double> speed) {

        double wantedSpeed = speed.isPresent() ? speed.get() : Drivetrain.kMaxSpeed;
        Pose2d current = m_swerve.m_odometry.getPoseMeters();
        Translation2d findXZ = new Translation2d(distance, current.getRotation());
        Transform2d convertToCorrectType = new Transform2d(findXZ, Rotation2d.fromDegrees(angle)); // already did
                                                                                                   // rotation above.
        Pose2d wanted = current.transformBy(convertToCorrectType);

        // technically, we'll want to break this up into xSpeed and ySpeed components.
        // However, we already know we want to go straight forward. So who cares?

        double xSpeed = wantedSpeed * Math.sin(angle);
        double ySpeed = wantedSpeed * Math.cos(angle);
        while (reachedFullTarget(wanted)) {
            m_swerve.drive(xSpeed, ySpeed, 0, false);
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                return false;
            }
        }
        return true;

    }

    public boolean driveForward(double distance, Optional<Double> speed) {
        double maxSpeed = Drivetrain.kMaxSpeed;
        double wantedSpeed = speed.isPresent() ? speed.get() : maxSpeed;

        Pose2d current = m_swerve.m_odometry.getPoseMeters();
        Translation2d findXZ = new Translation2d(distance, current.getRotation());
        Transform2d convertToCorrectType = new Transform2d(findXZ, new Rotation2d()); // already did rotation above.
        Pose2d wanted = current.transformBy(convertToCorrectType);

        while (reachedFullTarget(wanted)) {
            m_swerve.drive(wantedSpeed, 0, 0, false);
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                return false;
            }
        }
        return true;
    }

}