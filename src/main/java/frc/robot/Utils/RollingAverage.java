package frc.robot.Utils;

import java.util.LinkedList;
import java.util.Queue;

public class RollingAverage {

    private final Queue<Double> window = new LinkedList<>();
    private final int period;
    private double sum = 0.0;

    public RollingAverage(int period) {
        if (period <= 0) {
            throw new IllegalArgumentException("Period must be greater than 0");
        }
        this.period = period;
    }

    public final double getAverage(double value) {
        window.add(value);
        sum += value;

        // If the window is full, remove the oldest element
        if (window.size() > period) {
            sum -= window.remove();
        }

        // Return average based on current window size (handles warm-up period)
        return sum / window.size();
    }
}
