package frc.robot.subsystems.vision.calibration.spec;

import static edu.wpi.first.units.Units.Degrees;

public class Limelight3A extends CameraSpec {
    public Limelight3A() {
        super(
                "Limelight 3A",
                Degrees.of(54.5),
                Degrees.of(42)
        );
    }
}
