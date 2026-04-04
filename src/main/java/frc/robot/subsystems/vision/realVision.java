package frc.robot.subsystems.vision;

import java.util.Objects;
import java.util.Optional;
import java.util.stream.IntStream;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotPreferences;
import frc.robot.SystemManager;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightSettings.LEDMode;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;


import static edu.wpi.first.units.Units.DegreesPerSecond;

public class realVision implements aprilTagInterface{
    Limelight limelight1;
    Limelight limelight2;
    LimelightPoseEstimator ll1Estimate;
    LimelightPoseEstimator ll2Estimate;

    StructPublisher<Pose2d> ll1PosePublisher;
    StructPublisher<Pose2d> ll2PosePublisher;
    Optional<PoseEstimate> ll1Pose;
    Optional<PoseEstimate> ll2Pose;
    String m_chooser_current;

    Double ll1Timestamp = -1.0;
    Double ll2Timestamp = -1.0;

    Pose3d limelight1PoseCache;
    Pose3d limelight2PoseCache;

    /**
     * Creates limelights using the YALL library.
     *
     * Will be used to get vision estimations of where the robot is on the field when not in simulation
     *
     */
    public realVision() {

        limelight2 = new Limelight(Constants.limelightConstants.limelight2Name);
        limelight1 = new Limelight(Constants.limelightConstants.limelight1Name);

        limelight1PoseCache = RobotPreferences.getInstance().getLimelight1Pose();
        limelight2PoseCache = RobotPreferences.getInstance().getLimelight2Pose();



        limelight1.getSettings()
         .withLimelightLEDMode(LEDMode.PipelineControl)
         .withCameraOffset(RobotPreferences.getInstance().getLimelight1Pose())
         .save();
        
        limelight2.getSettings()
         .withLimelightLEDMode(LEDMode.PipelineControl)
         .withCameraOffset(RobotPreferences.getInstance().getLimelight2Pose())

         .save();


        ll1Estimate = limelight1.createPoseEstimator(EstimationMode.MEGATAG2);
        ll2Estimate = limelight2.createPoseEstimator(EstimationMode.MEGATAG2);

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        ll1PosePublisher = inst.getTable("SmartDashboard/Vision").getStructTopic("Limelight 1 Pose Estimation", Pose2d.struct).publish(PubSubOption.keepDuplicates(true));
        ll2PosePublisher = inst.getTable("SmartDashboard/Vision").getStructTopic("Limelight 2 Pose Estimation", Pose2d.struct).publish(PubSubOption.keepDuplicates(true));
        
        SendableChooser<String> m_chooser = new SendableChooser<>();

        m_chooser_current = Constants.limelightConstants.limelight1Name; // default option

        m_chooser.addOption(Constants.limelightConstants.limelight1Name, Constants.limelightConstants.limelight1Name);
        m_chooser.addOption(Constants.limelightConstants.limelight2Name, Constants.limelightConstants.limelight2Name);
        
        m_chooser.setDefaultOption(Constants.limelightConstants.limelight1Name, m_chooser_current);
        m_chooser.onChange((String limelightName)->{limelightForwarding(limelightName);});
        
        SmartDashboard.putData("Vision/" + Constants.limelightConstants.limelightToggleName, m_chooser);

        initLimelightForwarding();
    }


    @Override
    public void periodic() {
        if (!limelight1PoseCache.equals(RobotPreferences.getInstance().getLimelight1Pose())){
            limelight1.getSettings().withCameraOffset(RobotPreferences.getInstance().getLimelight1Pose()).save();
            limelight1PoseCache = RobotPreferences.getInstance().getLimelight1Pose();
        }

        if (!limelight2PoseCache.equals(RobotPreferences.getInstance().getLimelight2Pose())){
            limelight2.getSettings().withCameraOffset(RobotPreferences.getInstance().getLimelight2Pose()).save();
            limelight2PoseCache = RobotPreferences.getInstance().getLimelight2Pose();


        }

        limelight1.getData().getCamera2Robot();
        // These are called in periodic because they need to be according to YALL docs
        limelight2.getSettings().withRobotOrientation(
                new Orientation3d(
                    SystemManager.swerve.getRotation3d(),
                    new AngularVelocity3d(
                        DegreesPerSecond.of(0), // Roll, our robot will not be front flipping
                        DegreesPerSecond.of(0), // Pitch, our robot will not be rolling around in mud
                        DegreesPerSecond.of(SystemManager.swerve.getYawVelocity().baseUnitMagnitude()) // Yaw, rotation along the ground floor
                    )  
                )
            )
            .save();

        limelight1.getSettings().withRobotOrientation(
            new Orientation3d(
                SystemManager.swerve.getRotation3d(),
                new AngularVelocity3d(
                    DegreesPerSecond.of(0), // Roll, our robot will not be front flipping
                    DegreesPerSecond.of(0), // Pitch, our robot will not be rolling around in mud
                    DegreesPerSecond.of(SystemManager.swerve.getYawVelocity().baseUnitMagnitude()) // Yaw, rotation along the ground floor
                )
            )
         )
         .save();


    }
    
    /**
     * Gets the front limelight's estimated position. Returns null if there is no position or if there is no connection
     *
     * @return {@link Pose3d} of where the front limelight thinks the robot is
     * 
     */
    @Override
    public Pose3d getFrontPose() {
        ll1Pose = ll1Estimate.getAlliancePoseEstimate();
        if (ll1Pose.isEmpty()||ll1Pose.get().pose.equals(new Pose3d())) {
            SmartDashboard.putBoolean("Vision/Limelight 1 Read Correctly", false);
            ll1PosePublisher.set(new Pose2d());
            return null;
            
        }

        SmartDashboard.putBoolean("Vision/Limelight 1 Read Correctly", true);
        
        PoseEstimate poseEstimate = ll1Pose.get();
        ll1Timestamp = poseEstimate.timestampSeconds;
        ll1PosePublisher.set(poseEstimate.pose.toPose2d());
        return poseEstimate.pose;
    }

    /**
     * Gets the timestamp of the last found position of the front Limelight.
     *
     * @return {@link Double} of the timestamp of the last found position of the Limelight. Returns -1.0 if it cannot find a timestamp.
     */
    @Override
    public Double getFrontTimestamp() {
        Double timestamp;
        if (!ll1Pose.isEmpty()) {
            timestamp = ll1Timestamp;
            SmartDashboard.putNumber("Vision/Limelight 1 Timestamp", timestamp);
            SmartDashboard.putBoolean("Vision/Limelight 1 Connected", true);
        }
        else {
            timestamp = -1.0;
            SmartDashboard.putNumber("Vision/Limelight 1 Timestamp", timestamp);
            SmartDashboard.putBoolean("Vision/Limelight 1 Connected", false);
        }


        return timestamp;
    }

    /**
     * Gets the back Limelight's estimated position. Returns null if there is no position or if there is no connection
     *
     * @return {@link Pose3d} of where the front Limelight thinks the robot is
     * 
     */
    @Override
    public Pose3d getBackPose() {
        ll2Pose = ll2Estimate.getAlliancePoseEstimate();

        if (ll2Pose.isEmpty()||ll2Pose.get().pose.equals(new Pose3d())) {
            SmartDashboard.putBoolean("Vision/Limelight 2 Read Correctly", false);
            ll2PosePublisher.set(new Pose2d());
            return null;
        }

        SmartDashboard.putBoolean("Vision/Limelight 2 Read Correctly", true);


        PoseEstimate poseEstimate = ll2Pose.get();
        ll2Timestamp = poseEstimate.timestampSeconds;
        ll2PosePublisher.set(poseEstimate.pose.toPose2d());
        return poseEstimate.pose;
    }

    /**
     * Gets the timestamp of the last found position of the back Limelight.
     *
     * @return {@link Double} of the timestamp of the last found position of the Limelight. Returns -1.0 if it cannot find a timestamp.
     */
    @Override
    public Double getBackTimestamp() {
        Double timestamp;
        if (!ll2Pose.isEmpty()) {
            timestamp = ll2Timestamp;
            SmartDashboard.putNumber("Vision/Limelight 2 Timestamp", timestamp);
            SmartDashboard.putBoolean("Vision/Limelight 2 Connected", true);
        }
        else {
            timestamp = -1.0;
            SmartDashboard.putNumber("Vision/Limelight 2 Timestamp", timestamp);
            SmartDashboard.putBoolean("Vision/Limelight 2 Connected", false);
        }


        return timestamp;
    }


    public void limelightForwarding(String limelightSetting) {
        IntStream.range(5800, 5820).forEach(PortForwarder::remove);

        IntStream.range(5800, 5810).forEach(port -> {
            PortForwarder.add(port, Objects.equals(limelightSetting, Constants.limelightConstants.limelight1Name) ? "172.29.0.1" : "172.29.1.1", port);
        });

    }

    public void initLimelightForwarding() {
        IntStream.range(5800, 5810).forEach(port -> {
            PortForwarder.add(port, "172.29.0.1", port);
        });
    }

}
    