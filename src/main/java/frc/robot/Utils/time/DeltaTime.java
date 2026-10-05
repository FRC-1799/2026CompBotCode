package frc.robot.Utils.time;

public class DeltaTime {

    private long startTime;

    public final void Start() {
        startTime = System.nanoTime();
    }

    public final long DeltaNanoSec() {
        return (System.nanoTime() - startTime);
    }

    public final long DeltaMilliSec() {
        return (System.nanoTime() - startTime) / 1000000;
    }
}

