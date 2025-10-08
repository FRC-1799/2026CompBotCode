package frc.robot.subsystems.vision;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.BooleanArrayPublisher;
import edu.wpi.first.networktables.BooleanArraySubscriber;
import edu.wpi.first.networktables.BooleanArrayTopic;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.networktables.StructTopic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class realVision extends reefIndexerIO implements aprilTagInterface{
    
    public BooleanArraySubscriber[] coralLevelSubscribers;
    public BooleanArraySubscriber[] algaeLevelSubscribers;
    public BooleanArrayPublisher[] algaeLevelPublishers;
    public final StructSubscriber<Pose3d> robotFrontPoseSubscriber;
    public final StructSubscriber<Pose3d> robotBackPoseSubscriber;
    public final DoubleSubscriber robotFrontTimestampSubscriber;
    public final DoubleSubscriber robotBackTimestampSubscriber; 
    DoubleTopic robotFrontTimestampTopic;
    DoubleTopic robotBackTimestampTopic;
  



    public realVision() {
        // Default values just in case no values are grabbed
        boolean[] reefDefaultList = {false, false, false, false, false, false, false, false, false, false, false, false};
        boolean[] algaeDefaultList = {false, false, false, false, false, false};
        double timestampDefault = 0.0;
        Pose3d robotDefaultPose = new Pose3d();

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable visionTable = inst.getTable("Vision");
        NetworkTable coralPositionTable = visionTable.getSubTable("CoralPositions");

        coralLevelSubscribers = new BooleanArraySubscriber[]{
            coralPositionTable.getBooleanArrayTopic("CoralL1").subscribe(reefDefaultList),
            coralPositionTable.getBooleanArrayTopic("CoralL2").subscribe(reefDefaultList),
            coralPositionTable.getBooleanArrayTopic("CoralL3").subscribe(reefDefaultList),
            coralPositionTable.getBooleanArrayTopic("CoralL4").subscribe(reefDefaultList)
        };
        algaeLevelSubscribers = new BooleanArraySubscriber[]{
                inst.getBooleanArrayTopic("GUI/AlgaeL1").subscribe(algaeDefaultList),
                inst.getBooleanArrayTopic("GUI/AlgaeL2").subscribe(algaeDefaultList)
        };

        algaeLevelPublishers = new BooleanArrayPublisher[]{
            inst.getBooleanArrayTopic("GUI/AlgaeL1").publish(),
            inst.getBooleanArrayTopic("GUI/AlgaeL2").publish( )
    };



        // Gets the robot's position's subscriber
        StructTopic<Pose3d> robotFrontPoseTopic = visionTable.getStructTopic("FrontRobotPose", Pose3d.struct);
        robotFrontPoseSubscriber = robotFrontPoseTopic.subscribe(robotDefaultPose, PubSubOption.keepDuplicates(true));
        StructTopic<Pose3d> robotBackPoseTopic = visionTable.getStructTopic("BackRobotPose", Pose3d.struct);
        robotBackPoseSubscriber = robotBackPoseTopic.subscribe(robotDefaultPose, PubSubOption.keepDuplicates(true));

        // Gets the subscriber for the timestamp each position was published at
        robotFrontTimestampTopic = NetworkTableInstance.getDefault().getDoubleTopic("RobotPoseTimestampFront");
        robotFrontTimestampSubscriber = robotFrontTimestampTopic.subscribe(timestampDefault);
        robotBackTimestampTopic = NetworkTableInstance.getDefault().getDoubleTopic("RobotPoseTimestampBack");
        robotBackTimestampSubscriber = robotBackTimestampTopic.subscribe(timestampDefault);


    }
    
    @Override
    public Pose3d getFrontPose() {
        Pose3d robotFrontPose = robotFrontPoseSubscriber.get();
        if (robotFrontPose.equals(new Pose3d())){
            return null;
        }
        SmartDashboard.putBoolean("poseReadCorrectly", false);

        return robotFrontPose;
    }

    @Override
    public Double getFrontTimestamp() {
        SmartDashboard.putNumber("received front timestamp", robotFrontTimestampSubscriber.get());
        SmartDashboard.putBoolean("frontTimestampConnected", robotBackTimestampSubscriber.exists());

        return robotFrontTimestampSubscriber.get();
    }

    @Override
    public Double getBackTimestamp() {
        SmartDashboard.putNumber("received back timestamp", robotBackTimestampSubscriber.get());
        SmartDashboard.putBoolean("BackTimestampConnected", robotBackTimestampSubscriber.exists());
        return robotBackTimestampSubscriber.get();
    }

    @Override
    public Pose3d getBackPose() {
        Pose3d robotBackPose = robotBackPoseSubscriber.get();
        if (robotBackPose.equals(new Pose3d())){
            return null;
        }

        return robotBackPose;
    }

    @Override
    public boolean[][] getFullReefState() {
        boolean[][] reefArray = new boolean[12][4];
        for (int i=0;i<12;i++){
            reefArray[i] = new boolean[]{
                coralLevelSubscribers[0].get()[i],
                coralLevelSubscribers[1].get()[i],
                coralLevelSubscribers[2].get()[i],
                coralLevelSubscribers[3].get()[i]
            };
        }
        
        return reefArray;

    }

    @Override
    public boolean[][] getAlgaePosits() {
        
        boolean[][] algaeArray = new boolean[6][2];
        for (int i=0; i<6; i++){
            algaeArray[i] = new boolean[]{algaeLevelSubscribers[0].get()[i], algaeLevelSubscribers[0].get()[i]};
        }
        for (int i=0;i<2; i++){
            for (int j=0;j<6;j++){
                System.out.print(algaeArray[j][i]+ " ");
            }
            System.out.println();
        }
        System.out.println("\n");
        return algaeArray;
    }



    @Override
    public void freeAlgae(int row, int level) {
        BooleanArrayPublisher freeAlgaePublisher = algaeLevelPublishers[level];
        BooleanArraySubscriber freeAlgaeSubscriber = algaeLevelSubscribers[level];
        boolean[] algaeRowValues = freeAlgaeSubscriber.get();
        
        algaeRowValues[row] = false;

        freeAlgaePublisher.set(algaeRowValues);
    
        System.out.println(level);
        for (int j=0;j<6;j++){
            System.out.print(algaeRowValues[j]+ " ");
        }
        System.out.println("\n\n");
    

    }

    @Override
    public void resetSIMONLY() {
        throw new UnsupportedOperationException("Unimplemented method 'resetSIMONLY'");
    }
}
