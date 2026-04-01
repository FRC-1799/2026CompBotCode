package frc.robot.subsystems.vision.calibration.spec;

import edu.wpi.first.units.measure.Distance;

public class RobotSpec {
    public final Distance X;
    public final Distance Y;
    public final Distance Z;

    public RobotSpec(Distance length, Distance width, Distance height) {
        X = length;
        Y = width;
        Z = height;
    }
}
