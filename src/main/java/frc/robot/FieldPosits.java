package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class FieldPosits {
   
    public static final Translation3d blueHubTrans = new Translation3d(4.5974, 4.034536, 1.5748);
    public static final Translation3d redHubTrans= new Translation3d(11.938, 4.034536, 1.5748);

    public static final Pose2d blueHubPose = new Pose2d(blueHubTrans.toTranslation2d(), new Rotation2d());
    public static final Pose2d redHubPose = new Pose2d(redHubTrans.toTranslation2d(), new Rotation2d());

    
}
