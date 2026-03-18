package frc.robot.subsystems.vision.Calibration;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.*;
import java.util.stream.IntStream;

import static edu.wpi.first.units.Units.Degrees;

public class LimelightCal {

    private final List<LimelightCalTag> _relativeTags;
    private final List<String> _limelightIds;

    public LimelightCal(String[] limelightIds, LimelightCalTag[] relativeTagList) {
        _limelightIds = Arrays.stream(limelightIds).toList();
        _relativeTags = Arrays.stream(relativeTagList).toList();
    }

    public LimelightCal(String[] limelightIds, int tagCount, int startId, double distance, double spacing, double height) {
        if((tagCount & 1) == 0) throw new IllegalArgumentException("tagCount must be odd so there is a center tag");


        _limelightIds = Arrays.stream(limelightIds).toList();

        // only works if odd
        var outsideCount = (tagCount - 1)/2;


        _relativeTags = IntStream.range(outsideCount * -1, outsideCount).mapToObj(i -> {
            return new LimelightCalTag(startId - outsideCount + i, new Pose3d(i * spacing, height, distance, new Rotation3d()));
        }).toList();
    }

    public List<LimelightCalData> GetLimelightCalData() {
        return _limelightIds.stream().map(id -> {
            var fidList =  LimelightHelpers.getRawFiducials(id);
            return new LimelightCalData(id, fidList);
        }).toList();
    }
}