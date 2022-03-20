package frc.robot.swerveCode.util;
import java.util.HashMap;
import java.util.Optional;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;

public class AccelCoord implements Runnable {
    private HashMap<Double, Translation2d> accelMap = new HashMap<Double, Translation2d>();
    private double velocityX;
    private double velocityY;
    private double vNotX;
    private double vNotY;
    private double initTime;
    private AHRS navX;

    public AccelCoord(Translation2d initialVelocity, Translation2d desiredVelocity, AHRS superNav) {
        velocityX = desiredVelocity.getX();
        velocityY = desiredVelocity.getY();
        vNotX = initialVelocity.getX();
        vNotY = initialVelocity.getY();
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
        double displacementDueToAccelX = 0;
        double displacementDueToAccelY = 0;

        for (int i = 0; i < timeArray.length - 1; i++) {
            if (Math.abs(accelMap.get(timeArray[i]).getNorm()) >= 0.01) {
                if (initTime.isEmpty()) {
                    initTime = Optional.of(timeArray[i]);
                } else {
                    finalTime = Optional.of(timeArray[i]);
                }
            } else {
                double prevTime;
                if (i == 0) {
                    prevTime = 0;
                } else {
                    prevTime = timeArray[i-1];
                }
                double currentTime = timeArray[i];

                // Translates navX accel to terms of m_swerve accel (reflect accros y = -x)
                double accelXCurr = -accelMap.get(currentTime).getY();
                double accelYCurr = -accelMap.get(currentTime).getX();
                double accelXPrev = -accelMap.get(prevTime).getY();
                double accelYPrev = -accelMap.get(prevTime).getX();
                double delta = currentTime - prevTime;
                double jerkX = (accelXCurr + accelXPrev)/2;
                double jerkY = (accelYCurr + accelYPrev)/2;

                // Calculates displacement
                displacementDueToAccelX += (jerkX * delta * delta * delta)/6 + (accelXPrev * delta * delta)/2 + vNotX * delta;
                displacementDueToAccelY +=  (jerkY * delta * delta * delta)/6 + (accelYPrev * delta * delta)/2 + vNotX * delta;

                // Calculates velocity
                vNotX += (jerkX * delta * delta)/2 + accelXPrev * delta;
                vNotY += (jerkY * delta * delta)/2 + accelYPrev * delta;
            }
        }
        if (finalTime.isPresent() && initTime.isPresent()) {
            double displacementDueToVelocityX = (finalTime.get() - initTime.get()) * velocityX;
            double displacementDueToVelocityY = (finalTime.get() - initTime.get()) * velocityY;
            Translation2d returnValues = new Translation2d(displacementDueToVelocityX + displacementDueToAccelX, displacementDueToVelocityY + displacementDueToAccelY);
            return returnValues;
        } else {
            Translation2d returnValues=  new Translation2d(displacementDueToAccelX, displacementDueToAccelY);
            return returnValues;
        }
    }

    public Translation2d getEndingVelocity() {
        return new Translation2d(vNotX, vNotY);
    }
    @Override
    public void run() {
        updateAccel();
        
    }
}

