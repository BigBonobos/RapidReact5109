package frc.util.abstractions;

import java.util.List;
import java.util.Optional;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.locks.ReentrantLock;

import frc.robot.Robot;

public class AutonomousAbstraction extends AutoAbstract {

    /**
     * This lock may not be used if I limit threadPoolExecutor to one task.
     * Seems fine.
     * 
     * @deprecated
     */
    public static final ReentrantLock lock = new ReentrantLock();
    public static final ExecutorService fixedThreadPool = Executors.newSingleThreadExecutor();

    public Optional<CompletableFuture<Void>> runningMovement;

    public AutonomousAbstraction(Robot robo) {
        super(robo);
    }

    public void cancelMovement() {
        List<Runnable> stillRunning = fixedThreadPool.shutdownNow();
        if (!fixedThreadPool.isShutdown()) {
            if (this.runningMovement.isPresent()) {
                this.runningMovement.get().cancel(true);
            }

            for (Runnable run : stillRunning) {
                System.out.println("Shit. " + run.toString() + " failed to run");
            }
        }
       
    }

    /**
     * Rough autonomous drive forward. {@link frc.util.abstractions.AutoAbstract#driveForward(double, Optional)}
     * <p>Don't worry about the implementation here.</p>
     * <p>This inserts the drive movement into a separate threadpool, making it non-blocking.</p>
     * <p>Long story short, View the linked code above for actual implementation.
     * @return true if successful full run, false if blocked/failed to execute.
     */
    @Override
    public boolean driveForward(double distance, Optional<Double> speed) {
        boolean returnVal = lock.tryLock();
        if (returnVal) {
            CompletableFuture<Void> fut = 
                CompletableFuture.runAsync(() -> super.driveForward(distance, speed), fixedThreadPool);
                
            this.runningMovement = Optional.of(fut);
            try { fut.get(); } 
            catch (InterruptedException | ExecutionException e) { returnVal = false; }
        }
        this.runningMovement = null;
        lock.unlock();
        return returnVal;
    }

}
