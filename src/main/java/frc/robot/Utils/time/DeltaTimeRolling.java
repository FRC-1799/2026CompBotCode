package frc.robot.Utils.time;

import java.util.LinkedList;
import java.util.Queue;

public class DeltaTimeRolling {

    private long startTime;
    private final Queue<Double> window = new LinkedList<>();
    private int period;
    private double sum = 0.0;

    public DeltaTimeRolling(int period) {
        if (period <= 0) {
            throw new IllegalArgumentException("Period must be greater than 0");
        }
        this.period = period;
    }

    public final void Start() {
        startTime = System.nanoTime();
    }

    public final double DeltaNanoSec() {
        double value = (System.nanoTime() - startTime);

        window.add(value);
        sum += value;

        // If the window is full, remove the oldest element
        if (window.size() > period) {
            sum -= window.remove();
        }

        // Return average based on current window size (handles warm-up period)
        return sum / window.size();
    }

    public final double DeltaMilliSec() {
        return DeltaNanoSec() / 1000000;
    }
}
