package frc.robot.subsystems.vision.calibration;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

import static edu.wpi.first.units.Units.Degrees;

public class FiduciaryTag {
    int id;
    Pose3d position;

    Rotation3d cameraAngleTo = new Rotation3d();

    public FiduciaryTag(int id, Pose3d position) {
        this.id = id;
        this.position = position;
    }

    public double getXAngle() { return cameraAngleTo.getMeasureX().in(Degrees); }
    public double getYAngle() { return cameraAngleTo.getMeasureY().in(Degrees); }

    public double getZAngle() { return cameraAngleTo.getMeasureZ().in(Degrees); }

}
