package frc.robot;

import edu.wpi.first.networktables.*;
import java.lang.Math;

public class Limelight {

    // Initialization of variables
    public final NetworkTableInstance ntwrkInst;
    private final double targetSize;


    /**
     * Constructor for Limelight
     * @param targetSize The size of the longer side of the vision target(cm). For rapid react should be around 61.49125 cm.
     */
    public Limelight(double targetSize) {
        this.ntwrkInst = NetworkTableInstance.getDefault();
        this.targetSize = targetSize;
    }

    /**
     * Uses Limelight values and trig to find the distance the robot is from the goal
     * @return Either 0 if no vision tape is in frame or the distance the robot is from the vision target
     */
    public double[] calculate3dDistance(){

        // Gets the NetworkTable called limelight, containing the nessescary values
        NetworkTable limelight = ntwrkInst.getTable("limelight");
        boolean tv = limelight.getEntry("tv").getBoolean(false);
        double[] returnValues = new double[2];

        // Checks if limelight is returning values
        if (tv) {
            // Returns distance from goal (calculated using trig)
            double adjustedTlong = (100*targetSize)/(limelight.getEntry("tlong").getDouble(0)/420);
            double thirdDimension = adjustedTlong/Math.tan(59.7);
            double hypotenuseDistance = Math.sqrt((thirdDimension * thirdDimension) + (limelight.getEntry("tx").getDouble(0) * limelight.getEntry("tx").getDouble(0)));
            returnValues[0] = thirdDimension;
            returnValues[1] = hypotenuseDistance;

        } else {
            // Returns 0 if limelight isnt' returning values
            returnValues[0] = 0.0;
            returnValues[1] = 0.0;
        }

        return returnValues;
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
    public double calculateAngleOffset(){
        NetworkTable limelight = ntwrkInst.getTable("limelight");
        boolean tv = limelight.getEntry("tv").getBoolean(false);
        if(tv) {
            double zValue = calculate3dDistance()[0];
            double tx = limelight.getEntry("tx").getDouble(0);
            double theta = Math.atan(tx/zValue);
            return theta;
        } else  {
            return 0.0;
        }
    }
}
