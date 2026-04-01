package frc.robot.subsystems.vision.calibration.spec;

import edu.wpi.first.units.measure.Angle;

import static edu.wpi.first.units.Units.Degrees;

public abstract class CameraSpec {

    public final String name;
    public final CameraFov FOV;

    protected CameraSpec(String name, Angle horzFov, Angle vertFov) {
        this.name = name;
        this.FOV = new CameraFov(horzFov, vertFov);
    }

    protected CameraSpec() {
        this.name = "";
        this.FOV = new CameraFov(Degrees.of(0), Degrees.of(0));
    }
}
