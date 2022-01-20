package frc.robot;

import edu.wpi.first.networktables.*;
import java.lang.Math;

public interface Limelight {
    public NetworkTableInstance ntwrkInst = NetworkTableInstance.getDefault();

    public default double calculate3dDistance(){
        NetworkTable limelight = ntwrkInst.getTable("limelight");
        boolean tv = limelight.getEntry("tv").getBoolean(false);
        if (tv) {
            double adjustedTlong = (100*61.49125)/(limelight.getEntry("tlong").getDouble(0)/420);
            double thirdDimension = adjustedTlong/Math.tan(59.7);
            return thirdDimension;
        } else {
            return 0.0;
        }
    }

    public default double calculateAngleOffset(){
        NetworkTable limelight = ntwrkInst.getTable("limelight");
        boolean tv = limelight.getEntry("tv").getBoolean(false);
        if(tv) {
            double zValue = calculate3dDistance();
            double tx = limelight.getEntry("tx").getDouble(0);
            double theta = Math.atan(tx/zValue);
            return theta;
        } else  {
            return 0.0;
        }
    }
}
