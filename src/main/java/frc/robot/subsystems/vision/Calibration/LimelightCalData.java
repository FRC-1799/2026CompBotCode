package frc.robot.subsystems.vision.Calibration;

import java.util.Arrays;
import java.util.List;

public class LimelightCalData {

    public String id;
    public LimelightHelpers.RawFiducial[] fidList;

    public LimelightCalData(String id, LimelightHelpers.RawFiducial[] rawFiducials) {
        this.id = id;
        this.fidList = rawFiducials;
    }

    public List<LimelightHelpers.RawFiducial> getFidList() {
        return Arrays.asList(fidList);
    }
}