package frc.robot.subsystems.vision.calibration.spec;

import edu.wpi.first.units.measure.Angle;

public class CameraFov {
    public final Angle Horizontal;
    public final Angle Vertical;

    public CameraFov(Angle horizontal, Angle vertical) {
        Horizontal = horizontal;
        Vertical = vertical;
    }
}
