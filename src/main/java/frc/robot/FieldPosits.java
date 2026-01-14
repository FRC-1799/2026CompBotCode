package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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

    public static final Pose2d bottomCorner = new Pose2d(0,0, new Rotation2d());
    public static final Pose2d topCorner = new Pose2d(0, 8.069326, new Rotation2d());


    
}
