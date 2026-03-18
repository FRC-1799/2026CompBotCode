package frc.robot.subsystems.vision.Calibration;

import edu.wpi.first.math.geometry.Pose3d;

public class LimelightCalTag {
    public int id;
    public Pose3d pose;

    public LimelightCalTag(int id, Pose3d pose) {
        this.id = id;
        this.pose = pose;
    }


}
