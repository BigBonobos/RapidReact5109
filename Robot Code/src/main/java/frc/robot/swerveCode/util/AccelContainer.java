package frc.robot.swerveCode.util;
import java.util.Vector;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Notifier;

public class AccelContainer {
    public Vector<AccelCoord> coordStorage = new Vector<>();
    public Notifier accelCoordNotif;
    public AHRS navX;
    Translation2d absolutePosition = new Translation2d(0, 0);

    public AccelContainer(AHRS superNav) {
        navX = superNav;
    }

    public void appendAccelCoord(Translation2d desiredVelocity) {
        accelCoordNotif.stop();
        AccelCoord accelCoord = new AccelCoord(desiredVelocity, navX);
        accelCoordNotif = new Notifier(accelCoord);
        accelCoordNotif.startPeriodic(0.001);
        coordStorage.add(accelCoord);
    }

    public Translation2d getAbsolutePosTranslation2d() {
        accelCoordNotif.stop();
        for (AccelCoord coord: coordStorage) {
            absolutePosition.plus(coord.getDisplacement());
        }
        coordStorage.clear();
        return absolutePosition;
    }
}
