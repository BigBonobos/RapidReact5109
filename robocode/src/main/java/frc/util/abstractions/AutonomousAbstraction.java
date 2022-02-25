package frc.util.abstractions;

import java.util.List;
import java.util.Optional;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.locks.ReentrantLock;

import frc.robot.NewRobot;

public class AutonomousAbstraction extends AutoAbstract {

    /**
     * This lock may not be used if I limit threadPoolExecutor to one task. 
     * Seems fine.
     * @deprecated
     */
    public static final ReentrantLock lock = new ReentrantLock();
    public static final ExecutorService fixedThreadPool = Executors.newSingleThreadExecutor();

    public Optional<Future> runningMovement;

    public AutonomousAbstraction(NewRobot robo) {
        super(robo);
    }


    public void cancelMovement() {
        fixedThreadPool.shutdownNow();
        if (!fixedThreadPool.isShutdown());
        if (this.runningMovement.isPresent()) {
            this.runningMovement.get().cancel(true);
        }
    }

    /**
     * Rough autonomous drive forward.
     * @return 
     */
    @Override
    public boolean driveForward(double distance, Optional<Double> speed) {
        boolean ready = lock.tryLock();
        if (!ready) return false;
        this.runningMovement = Optional.of(fixedThreadPool.submit(() -> super.driveForward(distance, speed)));
        lock.unlock();
        return true;

        // try {
        //     CompletableFuture.runAsync(() -> super.driveForward(distance, speed), fixedThreadPool).get();
        // } catch (InterruptedException | ExecutionException e) {

        // }
    }

}
