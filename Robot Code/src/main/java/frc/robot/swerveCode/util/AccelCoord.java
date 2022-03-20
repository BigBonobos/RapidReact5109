package frc.robot.swerveCode.util;
import java.util.HashMap;
import java.util.Optional;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;

public class AccelCoord implements Runnable {
    private HashMap<Double, Translation2d> accelMap = new HashMap<Double, Translation2d>();
    private double velocityX;
    private double velocityY;
    private double initTime;
    private AHRS navX;

    public AccelCoord(Translation2d desiredVelocity, AHRS superNav) {
        velocityX = desiredVelocity.getX();
        velocityY = desiredVelocity.getY();
        navX = superNav;
        initTime = superNav.getLastSensorTimestamp();
    }

    public void updateAccel() {
        accelMap.put(navX.getLastSensorTimestamp() - initTime, new Translation2d(navX.getWorldLinearAccelX(), navX.getWorldLinearAccelY()));
    }


    public Translation2d getDisplacement() {
        Double[] timeArray = accelMap.keySet().toArray(Double[]::new);
        Optional<Double> initTime = Optional.empty();
        Optional<Double> finalTime = Optional.empty();
        double prevTime = 0;
        double displacementDueToAccelX = 0;
        double displacementDueToAccelY = 0;

        for (int i = 0; i < timeArray.length; i++) {
            if (Math.abs(accelMap.get(timeArray[i]).getNorm()) >= 0.01) {
                if (initTime.isEmpty()) {
                    initTime = Optional.of(timeArray[i]);
                } else {
                    finalTime = Optional.of(timeArray[i]);
                }
            } else {
                double currentTime = timeArray[i];
                double accelX = accelMap.get(currentTime).getX();
                double accelY = accelMap.get(currentTime).getY();
                double delta = currentTime - prevTime;
                displacementDueToAccelX += accelX * delta * delta;
                displacementDueToAccelY += accelY * delta * delta;
                prevTime = currentTime;
            }
        }
        if (finalTime.isPresent() && initTime.isPresent()) {
            double displacementDueToVelocityX = (finalTime.get() - initTime.get()) * velocityX;
            double displacementDueToVelocityY = (finalTime.get() - initTime.get()) * velocityY;
            return new Translation2d(displacementDueToVelocityX - displacementDueToAccelY, displacementDueToVelocityY - displacementDueToAccelX);
        } else {
            return new Translation2d(displacementDueToAccelX, displacementDueToAccelY);
        }
    }

    @Override
    public void run() {
        updateAccel();
        
    }
}

