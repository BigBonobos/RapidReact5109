package frc.robot.swerveCode.util;
import java.util.Optional;
import java.util.Vector;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Notifier;

public class AccelContainer {
    public Vector<AccelCoord> coordStorage = new Vector<>();
    public Optional<Notifier> accelCoordNotif = Optional.empty();
    public AHRS navX;
    private Translation2d currentVelocity = new Translation2d(0, 0);
    Translation2d absolutePosition = new Translation2d(0, 0);

    public AccelContainer(AHRS superNav) {
        navX = superNav;
    }

    public void appendAccelCoord(Translation2d desiredVelocity) {
        if (accelCoordNotif.isPresent()) {
            accelCoordNotif.get().stop();
            currentVelocity = coordStorage.get(coordStorage.size() - 1).getEndingVelocity();
        }
        AccelCoord accelCoord = new AccelCoord(currentVelocity, desiredVelocity, navX);
        Notifier accelCoordNotifUw = new Notifier(accelCoord);
        accelCoordNotifUw.startPeriodic(0.001);
        accelCoordNotif = Optional.of(accelCoordNotifUw);
        coordStorage.add(accelCoord);
    }
    public void resetOdometery() {
        currentVelocity = new Translation2d(0, 0);
        absolutePosition = new Translation2d(0, 0);
    }
    public Translation2d getAbsolutePosTranslation2d() {
        if (accelCoordNotif.isPresent()) {
            accelCoordNotif.get().stop();
        }
        for (AccelCoord coord: coordStorage) {
            absolutePosition.plus(coord.getDisplacement());
        }
        coordStorage.clear();
        return absolutePosition;
    }
}
