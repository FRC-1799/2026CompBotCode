package frc.robot.subsystems.vision.calibration;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.vision.calibration.spec.CameraSpec;

public class Camera {
    public final String id;

    public final CameraSpec spec;

    public Pose3d positionOnRobot;

    public final Pose3d testPositionOnRobot;

    public Camera(String id, CameraSpec spec) {
        this(id, spec, Pose3d.kZero);
    }
    public Camera(String id, CameraSpec spec, Pose3d testPositionOnRobot) {
        this.id = id;
        this.spec = spec;
        this.testPositionOnRobot = testPositionOnRobot;
    }


}
