package frc.robot.subsystems.lidar;

import java.util.LinkedList;
import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.FloatSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructArraySubscriber;
import edu.wpi.first.wpilibj.Timer;

public class realLidar implements lidarInterface{

    BooleanSupplier killSwitchSupplier = ()->false;


    protected NetworkTable lidarTable;
    protected StructArraySubscriber<Pose2d> hitboxes;
    protected FloatSubscriber nodeSideLen;






    public realLidar(){
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        lidarTable = inst.getTable("lidar");
        hitboxes = inst.getStructArrayTopic("individualReadings", Pose2d.struct).subscribe(null, PubSubOption.keepDuplicates(true));
        nodeSideLen = inst.getFloatTopic("NodeWidth").subscribe(0, PubSubOption.keepDuplicates(true));
    }


    @Override
    public List<Pair<Translation2d, Translation2d>> fetchObstacles() {
        Pose2d[] poses = hitboxes.get();
        Translation2d toTopRight = new Translation2d(nodeSideLen.get(), nodeSideLen.get());
        List<Pair<Translation2d,Translation2d>> hitboxReturn = new LinkedList<Pair<Translation2d, Translation2d>>();
        for (Pose2d pose: poses){
            hitboxReturn.add(new Pair<Translation2d, Translation2d>(pose.getTranslation(), pose.getTranslation().plus(toTopRight)));
        }
        return hitboxReturn;
    }

    @Override
    public boolean isConnected() {
        return Timer.getFPGATimestamp()-hitboxes.getLastChange()<1;
    }

    @Override
    public void setKillSwitchSupplier(BooleanSupplier killSwitchSupplier) {
        this.killSwitchSupplier=killSwitchSupplier;
    }

    @Override
    public boolean isAlive() {
        return !killSwitchSupplier.getAsBoolean();
    }
    
}
