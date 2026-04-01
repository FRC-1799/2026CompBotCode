package frc;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.vision.calibration.spec.Limelight3A;
import frc.robot.subsystems.vision.calibration.spec.RobotSpec;
import org.junit.jupiter.api.Test;
import frc.robot.subsystems.vision.calibration.Camera;
import frc.robot.subsystems.vision.calibration.CameraCal;

import java.util.Arrays;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Meter;
import static org.junit.jupiter.api.Assertions.*;

class CameraCalTest {

    Distance ApriltagSize = Inches.of(6.5);

    class LimelightSpec3A {
        Angle FovH = Degrees.of(54.5);
        Angle FovV = Degrees.of(42);
    }

    @Test
    void createLimeLightCal() {
        var cameras = Arrays.asList(
                new Camera(
                        "limelight",
                        new Limelight3A(),
                        new Pose3d(
                                0.25,
                                0.20,
                                0.3,
                                new Rotation3d(
                                        Degrees.of(0),
                                        Degrees.of(0),
                                        Degrees.of(20)
                                ))),
                new Camera(
                        "limelight",
                        new Limelight3A(),
                        new Pose3d(
                                0.25,
                                -0.20,
                                0.3,
                                new Rotation3d(
                                        Degrees.of(0),
                                        Degrees.of(0),
                                        Degrees.of(-20)
                                )))
        );


        CameraCal cal = new CameraCal(
                new RobotSpec(Meter.of(1), Meter.of(1), Meter.of(0.5)),
                cameras,
                Arrays.asList(1, 2, 3, 4, 5),
                2,
                0.25,
                0.5
        );

        var thrown = assertThrows(
                IllegalArgumentException.class,
                () -> new CameraCal(
                        new RobotSpec(Meter.of(1), Meter.of(1), Meter.of(0.5)),
                        cameras,
                        Arrays.asList(1, 2, 3, 4),
                        2,
                        0.25,
                        0.5
                ),
                "did not detect bad count"
        );
    }
}