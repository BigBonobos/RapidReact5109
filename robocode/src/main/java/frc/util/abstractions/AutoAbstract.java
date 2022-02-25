package frc.util.abstractions;

import java.util.Optional;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.swerveCode.RevOptimization;
import frc.robot.Drivetrain;
import frc.robot.NewRobot;

/**
 * Storage class for autonomous driving.
 * DO NOT USE THIS. THIS IS NOT THREAD SAFE.
 */
public abstract class AutoAbstract {

    public static final double acceptableErrorTranslationMeters = 0.1;
    public static final double acceptableErrorRotationDegrees = 3;

    public final NewRobot robot;

    public AutoAbstract(NewRobot robo) {
        robot = robo;
    }

    /**
     * Range of error: {@value #acceptableErrorTranslationMeters} 
     * <p>Change at {@link #acceptableErrorTranslationMeters}</p>
     * @param target Pose on field.
     * @return Whether or not goal is reached.
     */
    private boolean reachedTranslationTarget(Pose2d target) {
        return this.reachedTranslationTarget(target.getTranslation());
    }

    /**
     * Range of error: {@value #acceptableErrorTranslationMeters} 
     * <p>Change at {@link #acceptableErrorTranslationMeters}</p>
     * @param target Translation goal.
     * @return Whether or not goal is reached.
     */
    private boolean reachedTranslationTarget(Translation2d target) {
        Pose2d current = robot.drivetrainModule.drivetrainOdometry.getPoseMeters();
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
    private boolean reachedRotationTarget(Pose2d target) {
        return this.reachedRotationTarget(target.getRotation());
    }

    /**
     * Range of error: {@value #acceptableErrorRotationDegrees} 
     * <p>Change at {@link #acceptableErrorRotationDegrees}</p>
     * @param target Rotation target.
     * @return Whether or not goal is reached.
     */
    private boolean reachedRotationTarget(Rotation2d target) {
        Pose2d current = robot.drivetrainModule.drivetrainOdometry.getPoseMeters();
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
    private boolean reachedFullTarget(Pose2d target) {
        return this.reachedRotationTarget(target) && this.reachedTranslationTarget(target);
    }

    // public void translateToFieldPoint(Translation2d fieldPoint, Rotation2d absRotation, double speed, double rotSpeed) {
    //     this.translateToFieldPoint(new Pose2d(fieldPoint, absRotation), speed, rotSpeed);
    // }

    public void translateTo(Translation2d fieldPoint, Optional<Double> speed) {

        double current = robot.drivetrainModule.navX.getAngle();
        Translation2d delta = fieldPoint.minus(robot.drivetrainModule.drivetrainOdometry.getPoseMeters().getTranslation());
        double yaw =  Math.atan2(delta.getX(), delta.getY());
        // double wanted = RevOptimization.placeInAppropriate0To360Scope(current, yaw);
        double wantedSpeed = speed.isPresent() ? speed.get() : Drivetrain.kMaxAngularSpeed;


        double xSpeed = wantedSpeed * Math.cos(yaw);
        double ySpeed = wantedSpeed * Math.sin(yaw);

        while (!reachedTranslationTarget(fieldPoint)) {
            
        }
        // this.translateToFieldPoint(new Pose2d(fieldPoint, new Rotation2d()), speed, 0);
    }

    public void rotateTo(Rotation2d absRotation, Optional<Double> rotSpeed) {
        double current = robot.drivetrainModule.navX.getAngle();
        double wanted = RevOptimization.placeInAppropriate0To360Scope(current, absRotation.getDegrees());
        double wantedSpeed = rotSpeed.isPresent() ? rotSpeed.get() : Drivetrain.kMaxAngularSpeed;


        while (!reachedRotationTarget(absRotation)) {

            // I may be using this incorrectly. :thumbsup:
            ChassisSpeeds test = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, wantedSpeed, Rotation2d.fromDegrees(wanted));
            robot.drivetrainModule.driveChassisSpeed(test);
        }
        
      
        
    }

    /**
     * Wasteful variable assignments are for code readability. I'm sorry, Rob. I cannot optimize this.
     * 
     * @param fieldPose
     *            Wanted position on the field. Both translation and rotation.
     * @param speed
     *            Optional speed wanted. Defaults to max.
     */
    public void translateToFieldPoint(Pose2d fieldPose, Optional<Double> speed, Optional<Double> rotSpeed) {
        double wantedSpeed = speed.isPresent() ? speed.get() : Drivetrain.kMaxSpeed;

        while (reachedFullTarget(fieldPose)) {
            boolean trans = reachedTranslationTarget(fieldPose);
            boolean rot = reachedRotationTarget(fieldPose);

            if (!trans) {

            }
            robot.drivetrainModule.drive(wantedSpeed, 0, 0, false);
            try { Thread.sleep(10); } catch (InterruptedException e) { break; }
        }

    }




    public void driveForward(double distance, Optional<Double> speed) {
        double maxSpeed = Drivetrain.kMaxSpeed;
        double wantedSpeed = speed.isPresent() ? speed.get() : maxSpeed;

        Pose2d current = robot.drivetrainModule.drivetrainOdometry.getPoseMeters();
        Translation2d findXZ = new Translation2d(distance, current.getRotation());
        Transform2d convertToCorrectType = new Transform2d(findXZ, new Rotation2d()); // already did rotation above.
        Pose2d wanted = current.transformBy(convertToCorrectType);

        while (reachedFullTarget(wanted)) {
            robot.drivetrainModule.drive(wantedSpeed, 0, 0, false);
            try { Thread.sleep(10); } catch (InterruptedException e) { break; }
        }
    }

}
