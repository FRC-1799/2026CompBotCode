package frc.robot.subsystems.vision.Calibration;


import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.SystemManager;

public class configureState extends Command{

    StructPublisher<Pose2d> ll1CalPose;
    StructPublisher<Pose2d> ll2CalPose;

    public configureState() {
        addRequirements(SystemManager.swerve);
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        ll1CalPose = inst.getTable("SmartDashboard/LimelightCal").getStructTopic("ll1CalPose", Pose2d.struct).publish(PubSubOption.keepDuplicates(true));
        ll2CalPose = inst.getTable("SmartDashboard/LimelightCal").getStructTopic("ll2CalPose", Pose2d.struct).publish(PubSubOption.keepDuplicates(true));

    }

    @Override
    public void execute() {
        double[] ll1TargetPose = LimelightHelpers.getTargetPose_CameraSpace(Constants.limelightConstants.limelight1Name);
        double[] ll2TargetPose = LimelightHelpers.getTargetPose_CameraSpace(Constants.limelightConstants.limelight2Name);

        Translation2d botTranslate = SystemManager.swerve.getPose().getTranslation();



        // Pose of April Tag from Limelight 1's estimation
        if(ll1TargetPose.length == 6) {
            Translation3d camTranslatell1 = new Translation3d(ll1TargetPose[2], ll1TargetPose[0] * -1, ll1TargetPose[1]);
            Rotation3d camRotll1 = new Rotation3d(Degrees.of(ll1TargetPose[5] * -1), Degrees.of(ll1TargetPose[3]), Degrees.of(ll1TargetPose[4] * -1));


            Translation2d fidPostll1 = botTranslate.minus(camTranslatell1.toTranslation2d());

            Pose2d ll1Pose = new Pose2d(fidPostll1, camRotll1.toRotation2d());
            SmartDashboard.putNumberArray("Limelight1-tpp-raw", ll1TargetPose);
            ll1CalPose.set(ll1Pose);
        }

        // Pose of April Tag from Limelight 2's estimation
        if(ll2TargetPose.length == 6) {
            Translation3d camTranslatell2 = new Translation3d(ll2TargetPose[2], ll2TargetPose[0] * -1, ll2TargetPose[1]);
            Rotation3d camRotll2 = new Rotation3d(Degrees.of(ll2TargetPose[5] * -1), Degrees.of(ll2TargetPose[3]), Degrees.of(ll2TargetPose[4] * -1));

            Translation2d fidPostll2 = botTranslate.minus(camTranslatell2.toTranslation2d());

            Pose2d ll2Pose = new Pose2d(fidPostll2, camRotll2.toRotation2d());
            SmartDashboard.putNumberArray("Limelight2-tpp-raw", ll2TargetPose);
            ll2CalPose.set(ll2Pose);
        }
    }
}