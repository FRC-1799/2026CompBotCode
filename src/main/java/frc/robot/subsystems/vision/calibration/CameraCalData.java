package frc.robot.subsystems.vision.calibration;

import frc.robot.subsystems.vision.LimelightHelpers;

import java.util.List;

public class CameraCalData {

    public String id;
    public List<LimelightHelpers.LimelightTarget_Fiducial> fidList;

    public CameraCalData(String id, List<LimelightHelpers.LimelightTarget_Fiducial> rawFiducials) {
        this.id = id;
        this.fidList = rawFiducials;
    }
}
