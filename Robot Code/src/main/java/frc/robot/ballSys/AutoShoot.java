package frc.robot.ballSys;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Robot;

public class AutoShoot {

    public final Translation2d goal;
    public final double[] goalVec;
    public final double goalRadius;
    public final Robot robot;
    public final int fps;
    public final double invFps;
    public final double initProjectileSpeed;
    public final double gravity;

    public static final Rotation2d shooterPitch = Rotation2d.fromDegrees(70);
    public static final int maxFrames = 300;
    public static final double shooterHeight = 0.25;
    public static final double goalLipHeight = 2.4384;
    // public static final double goalLipRAdius = 0.9144;

    public AutoShoot(Robot robo, Translation2d goa, double goalRad, int framesPerSecond) {
        robot = robo;
        goal = goa;
        goalVec = new double[] { goa.getX(), goalLipHeight, goa.getY() };
        goalRadius = goalRad;
        fps = framesPerSecond;
        invFps = 1d / fps;

        gravity = 9.81;
        initProjectileSpeed = 7.84;
    }

    private Translation2d getOriginTranslation() {
        return new Translation2d(robot.m_swerve.navX.getDisplacementX(), robot.m_swerve.navX.getDisplacementY());
    }

    public Rotation2d getYawToGoal() {
        final Translation2d origin = getOriginTranslation();
        final double xDistance = goal.getX() - origin.getX();
        final double zDistance = goal.getY() - origin.getY();
        final double yaw = Math.atan2(-xDistance, -zDistance);
        return new Rotation2d(yaw);
    }

    public double[] getDirFromYawPitchSpeed(final Rotation2d yaw, final Rotation2d pitch, final double speed) {
        return getDirFromYawPitchSpeed(yaw.getRadians(), pitch.getRadians(), speed);
    }

    /**
     * This may be broken.
     * 
     * @param yaw
     * @param pitch
     * @param speed
     * @return
     */
    public double[] getDirFromYawPitchSpeed(final double yaw, final double pitch, final double speed) {
        final double thetaY = Math.PI + yaw;
        final double thetaP = pitch;
        final double x = speed * Math.sin(thetaY);
        final double y = speed * Math.sin(thetaP);
        final double z = speed * Math.cos(thetaY);
        final double VxMag = Math.sqrt(x * x + z * z);
        final double VxRatio = Math.sqrt(VxMag * VxMag - y * y);
        final double allRatio = VxRatio / VxMag;
        return new double[] { x * allRatio, y, z * allRatio };
    }

    private double[] add(double[] pos, final double[] vel) {
        return add(pos, vel, 1);
    }

    private double[] add(double[] pos, final double[] vel, double scale) {
        return new double[] {
            pos[0] + vel[0] * scale,
            pos[1] + vel[1] * scale,
            pos[2] + vel[2] * scale,
        };
    }

    private boolean madeItToGoal(final double[] current, final double[] next) {
        final boolean flag1 = Math.sqrt(Math.pow(current[0], 2) + Math.pow(current[2], 2)) < goalRadius;
        final boolean flag2 = Math.sqrt(Math.pow(next[0], 2) + Math.pow(next[2], 2)) < goalRadius;
        final boolean flag3 = current[1] > goalVec[1] && next[1] < goalVec[1];
        return flag1 && flag2 && flag3;
    }

    public boolean calculateIfShotGood(Translation2d origin, Rotation2d yaw) {
        double[] currentPos = new double[] { origin.getX(), shooterHeight, origin.getY() };
        double[] vel = getDirFromYawPitchSpeed(yaw, shooterPitch, initProjectileSpeed);
        double[] nextPos = add(currentPos, vel);
        double[] offsets = new double[3];
        final double airResistanceXZ = 0;
        final double airResistanceY = 0;
        int frames = 1;

        while (frames < maxFrames) {

            offsets[0] = -vel[0] * airResistanceXZ;
            offsets[1] = -vel[1] * airResistanceY - gravity;
            offsets[2] = -vel[2] * airResistanceXZ;

            if (madeItToGoal(currentPos, nextPos)) {
                return true;
            }

            if (vel[1] < 0 && nextPos[1] < 0) {
                return false;
            }

            currentPos = add(currentPos, vel, invFps);
            vel = add(vel, offsets, invFps);
            nextPos = add(currentPos, vel, invFps);
        }

        frames += 1;
        return false;
    }

    public Translation2d getTranslation(Translation2d origin, double distanceOffset, Rotation2d yaw) {
        return new Translation2d(
                origin.getX() + (distanceOffset * Math.sin(yaw.getRadians())),
                origin.getY() + (distanceOffset * Math.cos(yaw.getRadians())));

    }

    public Translation2d findShootablePos(final double incrementDistance, final Rotation2d wantedYaw) throws Exception {
        final Translation2d origin = getOriginTranslation();
        Translation2d testPos = origin;
        boolean success = false;

        do {

            System.out.println(testPos);
            success = calculateIfShotGood(testPos, wantedYaw);
            if (success) return testPos;
            testPos = getTranslation(testPos, incrementDistance, wantedYaw);

        } while (origin.getDistance(goal) > origin.getDistance(testPos));
        Rotation2d newWanted = wantedYaw.plus(Rotation2d.fromDegrees(180));
        do {
            System.out.println(testPos);
            System.out.println(origin.getDistance(testPos));
            success = calculateIfShotGood(testPos, newWanted);
            if (success) return testPos;
            testPos = getTranslation(testPos, incrementDistance, newWanted);
        } while (origin.getDistance(testPos) < 20);;

        throw new Exception("Fuck lol, can't find a good shot.");
    }

    // public boolean shootAtGoal() {

    //     // if (robot.ballFondler.getAndUpdateBallCount() == 0) {
    //     // return false;
    //     // }

    //     Rotation2d wantedYaw = getYawToGoal();
    //     try {
    //         Translation2d wantedPos = findShootablePos(0.25, wantedYaw);
    //         System.out.printf("Wanted pos: %s", wantedPos.toString());
    //         // robot.m_swerve.auto.translateTo(wantedPos);
    //         robot.m_swerve.auto.rotateTo(wantedYaw);
    //         robot.ballFondler.shoot();
    //         return true;

    //     } catch (Exception e) {
    //         e.printStackTrace();
    //         return false;
    //     }

    // }

}
