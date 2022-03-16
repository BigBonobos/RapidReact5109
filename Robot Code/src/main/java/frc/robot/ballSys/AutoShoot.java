package frc.robot.ballSys;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Robot;

public class AutoShoot  {
    

    public double shooterDegrees;


    public final Translation2d goal;
    public final Robot robot;


    public AutoShoot(Robot robo, Translation2d goa) {
        robot = robo;
        goal = goa;
    }

    private Translation2d getOriginTranslation() {
        return new Translation2d(robot.m_swerve.navX.getDisplacementX(), robot.m_swerve.navX.getDisplacementY());
    }


    public Rotation2d getYawToGoal(Translation2d destination) {
        Translation2d origin = getOriginTranslation();
        double xDistance = destination.getX() - origin.getX();
        double zDistance = destination.getY() - origin.getY();
        double yaw = Math.atan2(-xDistance, -zDistance);
        return new Rotation2d(yaw);
    }

    /**
     * This may be broken.
     * @param yaw
     * @param pitch
     * @param speed
     * @return
     */
    public double[] getDirFromYawPitchSpeed(double yaw, double pitch, double speed) {
        double thetaY = Math.PI + yaw;
        double thetaP = pitch;

        double x = speed * Math.sin(thetaY);
        double y = speed * Math.sin(thetaP);
        double z = speed * Math.cos(thetaY);
        double VxMag = Math.sqrt(x * x + z * z);
        double VxRatio = Math.sqrt(VxMag * VxMag - y * y);
        double allRatio = VxRatio / VxMag;

        return new double[] { x * allRatio, y, z * allRatio};


    }

  



    public void calculateIfShotGood() {

        Translation2d origin = getOriginTranslation();
        Rotation2d yaw = getYawToGoal(goal);
        double[] currentPos = new double[]{ origin.getX(), 0, origin.getY()};

        // get projectile speed, put it in here.
        double[] vel = getDirFromYawPitchSpeed(yaw.getRadians(), shooterDegrees, 0);
        double[] nextPos = new double[]{ origin.getX() + vel[0], vel[1], origin.getY() + vel[2]};


        double gravity = 9.81;



        let currentPosition = this.initialPos.clone();
        let currentVelocity = this.initialVel.clone();
        let nextPosition = currentPosition.clone().add(currentVelocity);
        let hitPos: Vec3 | null = null;
        let block: Block | null = null;

        let totalTicks = 0;
        const gravity = this.gravity; // + this.gravity * airResistance.y;
        let offsetX: number = -currentVelocity.x * airResistance.h;
        let offsetY: number = -currentVelocity.y * airResistance.y - gravity;
        let offsetZ: number = -currentVelocity.z * airResistance.h;

        while (totalTicks < 300) {
            totalTicks += 1;
            offsetX = -currentVelocity.x * airResistance.h;
            offsetY = -currentVelocity.y * airResistance.y - gravity;
            offsetZ = -currentVelocity.z * airResistance.h;

            if (blockChecking && this.interceptCalcs) {
                block = this.interceptCalcs.check(currentPosition, nextPosition)?.block;
            }

         


            if (currentVelocity.y < 0 && currentPosition.y < 0) break;

            currentPosition.add(currentVelocity);
            currentVelocity.translate(offsetX, offsetY, offsetZ);
            nextPosition.add(currentVelocity);
        }



    }

    public void shootAtGoal(Translation2d destination, double wantedSpeed) {
        Translation2d origin = getOriginTranslation();


        // // rotate to wanted yaw.
        // robot.m_swerve.auto.rotateTo(absRot);






    }


}


