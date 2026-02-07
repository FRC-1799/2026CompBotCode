package frc.robot;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class FieldPosits {
   
    // public static final Translation3d blueHubTrans = new Translation3d(4.5974, 4.034536, 1.5748);
    // public static final Translation3d redHubTrans= new Translation3d(11.938, 4.034536, 1.5748);


    // public static final Pose3d blueHubPose3d = new Pose3d(blueHubTrans, new Rotation3d());
    // public static final Pose3d redHubPose3d = new Pose3d(redHubTrans, new Rotation3d());

    // public static final Pose2d blueHubPose2d = blueHubPose3d.toPose2d();
    // public static final Pose2d redHubPose2d = redHubPose3d.toPose2d();
    public static final Translation3d hubTrans = new Translation3d(4.5974, 4.034536, 1.5748);


    public static final Pose3d hubPose3d = new Pose3d(hubTrans, new Rotation3d());

    public static final Pose2d hubPose2d = hubPose3d.toPose2d();

    public static final Translation2d mid = new Translation2d(8.25, 4);

    public static final Translation2d depo = new Translation2d(0.414, 6);

    public static final Pose2d outpost = new Pose2d(0.382, 0.650, new Rotation2d());

    public static final Pose2d bottomAllianceDSCorner = new Pose2d(0,0, new Rotation2d());
    public static final Pose2d topAllianceDSCorner = new Pose2d(0, 8.0, new Rotation2d());
    public static final Collection<Pose2d> passingPoses = List.of(bottomAllianceDSCorner, topAllianceDSCorner);


    public static final Pose2d topAllianceMidCorner = new Pose2d(4, 8, new Rotation2d());
    public static final Pose2d bottomAllianceMidCorner = new Pose2d(4, 0, new Rotation2d());

    public static final Pose2d bottomScorePoseAuto = new Pose2d(4, 0.5, new Rotation2d());
    public static final Pose2d topScorePoseAuto = new Pose2d(4, 7.5, new Rotation2d());

    // public static final Pose2d bottomScorePoseAuto = new Pose2d(3.646, 4, new Rotation2d());
    // public static final Pose2d topScorePoseAuto = new Pose2d(3.646, 4, new Rotation2d());
    public static final Collection<Pose2d> scoringPoses= List.of(bottomScorePoseAuto, topScorePoseAuto);

    public static final Pose2d topIntakePassoffMid = new Pose2d(5.67, 7.5, new Rotation2d());
    public static final Pose2d bottomIntakePassoffMid = new Pose2d(5.67, 0.5, new Rotation2d());
    public static final Collection<Pose2d> intakingHandoffPoses= List.of(topIntakePassoffMid, bottomIntakePassoffMid);

    public static final Rectangle2d alianceZone = new Rectangle2d(bottomAllianceDSCorner.getTranslation(), topAllianceMidCorner.getTranslation());




    
}
