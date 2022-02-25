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
public abstract class AutoAbstract extends MovementUtil {


    public AutoAbstract(NewRobot robo) {
        super(robo);
    }

    public void translateTo(Translation2d fieldPoint, Optional<Double> speed) {

        // double current = robot.drivetrainModule.navX.getAngle();
        // double wanted = RevOptimization.placeInAppropriate0To360Scope(current, yaw);

        Translation2d delta = fieldPoint.minus(robot.drivetrainModule.drivetrainOdometry.getPoseMeters().getTranslation());
        double yaw =  Math.atan2(delta.getX(), delta.getY());
        double wantedSpeed = speed.isPresent() ? speed.get() : Drivetrain.kMaxAngularSpeed;

        double xSpeed = wantedSpeed * Math.cos(yaw);
        double ySpeed = wantedSpeed * Math.sin(yaw);

        while (!reachedTranslationTarget(fieldPoint)) {
            robot.drivetrainModule.drive(xSpeed, ySpeed, 0, false);
            try { Thread.sleep(10); } catch (InterruptedException e) { break; }
            
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
            try { Thread.sleep(10); } catch (InterruptedException e) { break; }
        }
        
      
        
    }

    /**
     * Wasteful variable assignments are for code readability. I'm sorry, Rob. I cannot optimize this.
     * <p>Also, this does translation, then rotation. It does not run it together.</p>
     * I can fix that later.
     * 
     * @param fieldPose
     *            Wanted position on the field. Both translation and rotation.
     * @param speed
     *            Optional speed wanted. Defaults to max.
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




    public boolean driveForward(double distance, Optional<Double> speed) {
        double maxSpeed = Drivetrain.kMaxSpeed;
        double wantedSpeed = speed.isPresent() ? speed.get() : maxSpeed;

        Pose2d current = robot.drivetrainModule.drivetrainOdometry.getPoseMeters();
        Translation2d findXZ = new Translation2d(distance, current.getRotation());
        Transform2d convertToCorrectType = new Transform2d(findXZ, new Rotation2d()); // already did rotation above.
        Pose2d wanted = current.transformBy(convertToCorrectType);

        while (reachedFullTarget(wanted)) {
            robot.drivetrainModule.drive(wantedSpeed, 0, 0, false);
            // Thread.sleep(10);
            try { Thread.sleep(10); } catch (InterruptedException e) { return false; }
        }
        
        return true;
    }

}
