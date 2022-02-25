package frc.util.abstractions;

import java.util.Optional;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.locks.ReentrantLock;

import frc.robot.NewRobot;

public class AutonomousAbstraction extends AutoAbstract {
    public static final ReentrantLock lock = new ReentrantLock();
    public static final ExecutorService fixedThreadPool = Executors.newFixedThreadPool(1);

    public AutonomousAbstraction(NewRobot robo) {
        super(robo);

    }

    /**
     * Rough autonomous drive forward.
     */
    @Override
    public void driveForward(double distance, Optional<Double> speed) {
        boolean ready = lock.tryLock();
        if (!ready)
            return;

        try {
            CompletableFuture.runAsync(() -> super.driveForward(distance, speed), fixedThreadPool).get();
        } catch (InterruptedException | ExecutionException e) {
        }
        lock.unlock();

    }

}
