package frc.robot.pneumaticsTesting;

public abstract class ISolenoid {
    public final int forwardPort;
    public final int reversePort;


    protected ISolenoid(int forwardPort, int reversePort) {
        this.forwardPort = forwardPort;
        this.reversePort = reversePort;
    }

    public abstract void extendFully();


    public abstract void retractFully();

    public abstract boolean toggle();

    public abstract void relax();


}
