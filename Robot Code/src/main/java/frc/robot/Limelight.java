package frc.robot;

import edu.wpi.first.networktables.*;
import java.lang.Math;
import java.util.OptionalDouble;

public class Limelight {

    // Initialization of variables
    private NetworkTableInstance ntwrkInst;
    private double targetSize;
    private final static double limelightFOV = 59.6;
    public NetworkTable limelight;


    /**
     * Constructor for Limelight
     * @param targetSize The size of the longer side of the vision target(cm). For rapid react should be around 61.49125 cm.
     */
    public Limelight(double targetSizeParam) {
        ntwrkInst = NetworkTableInstance.getDefault();
        targetSize = targetSizeParam;
        limelight = ntwrkInst.getTable("limelight");
    }

    /**
     * Uses Limelight values and trig to find the distance the robot is from the goal
     * @return Either 0 if no vision tape is in frame or the distance the robot is from the vision target
     */
    public double calculateYDistance(){
        final double limelightHeightInches = 22;
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry ty = table.getEntry("ty");
        double targetOffsetAngle_Vertical = ty.getDouble(0.0);
        
        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = 25.0;
        
        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = 20.0;
        
        // distance from the target to the floor
        double goalHeightInches = 60.0;
        
        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
        
        //calculate distance
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightHeightInches)/Math.tan(angleToGoalRadians);
        return distanceFromLimelightToGoalInches;
    }

    /**
     * Calculates the angle of the vision target relative to the bot
     * @return Either 0 or angle offset (theta, degrees)
     * <ul>
     * <li>0 degrees is bot facing target</li>
     * <li>-theta means the target is to the left of the bot</li>
     * <li>+theta means the target is to the right of the bot</li>
     * </ul>
     */
    public OptionalDouble getXOffset() {
        boolean tv = limelight.getEntry("tv").getBoolean(true);

        if (tv) {
            return OptionalDouble.of(limelight.getEntry("tx").getDouble(0));
        } else {
            return OptionalDouble.empty();
        }
    }
}
