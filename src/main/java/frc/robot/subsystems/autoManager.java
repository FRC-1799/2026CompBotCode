package frc.robot.subsystems;

import java.io.FileReader;
import java.io.IOException;

import java.util.LinkedList;
import java.util.List;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.pathfinding.LocalADStar;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.FieldPosits;
import frc.robot.SystemManager;
import frc.robot.FieldPosits.reefLevel;
import frc.robot.FieldPosits.reefPole;
import frc.robot.FieldPosits.reefLevel.algaeRemovalEnum;
import frc.robot.Utils.LocalAStar;
import frc.robot.Utils.utilFunctions;

import frc.robot.commands.auto.spin;



public class autoManager{

    public static boolean hasControl=false;
    public static BooleanSupplier hasControlSupplier=null;
    public static Command currentRoutine=null;
    public static LocalAStar map = new LocalAStar();

    //static int resetCount=0;
    public static int cycleCount=0;


    public static int score=0;



    /**initializes the auto manager. this must be called before the auto manager is used */
    public static void autoManagerInit() {

        
    }


    /**handles the periodic tasks of the auto manager. should be called every cycle */
    public static void periodic(){
        SmartDashboard.putNumber("Cycle count", cycleCount);
        SmartDashboard.putNumber("Score", score);


        if (currentRoutine!=null){
            if (!currentRoutine.isScheduled()){
                currentRoutine=null;
            }
        }


        if (hasControlSupplier!=null){
            if (hasControlSupplier.getAsBoolean()!=hasControl){
                if (hasControlSupplier.getAsBoolean()==false){
                    takeControl();
                }
                else{
                    giveControl();
                }
            }
        }


        if (hasControl){
            if (currentRoutine==null || currentRoutine.getClass()==spin.class){
                currentRoutine=getAutoAction();
                currentRoutine.schedule();
            }
        }

        if (currentRoutine==null){
            SmartDashboard.putString("autoRoutine", "null");
            
        }
        else{
            SmartDashboard.putString("autoRoutine", currentRoutine.getName());
        }
    }


    /**
     * swaps wether or not the auto manager has control
     * @param isGift wether or not the auto manager should gain control
     */
    public static void swapControl(boolean isGift){
        if (isGift){
            giveControl();
        }
        else{
            takeControl();
        }
    }

    /**gives the auto manager control */
    public static void giveControl(){
        hasControl=true;
    }

    /**takes control away from the auto manager */
    public static void takeControl(){
        hasControl=false;
        if (currentRoutine!=null){
            currentRoutine.cancel();
        }
    }

    /**
     * sets a supplier that the auto manager will use to determine if it has control. Note if this function is used the giveControl and take control methods may not work as intended.
     * @param supplier the supplier to determine wether the auto manager has control
     */
    public static void setControlBooleanSupplier(BooleanSupplier supplier){
        hasControlSupplier=supplier;    
    }


    /**
     * resets the internal A* map and recalculates paths using the starting pose
     * @param startingPose the pose to start the pathplanner on
     */
    protected static void resetMap(Pose2d startingPose){
        map.resetMap();
        map.getMapPoint(startingPose).start();
    }









    /** @return the best auto action to take at the frame called in the form of a command*/
    public static Command getAutoAction(){
        
        return new spin();
        

    }

 


}
