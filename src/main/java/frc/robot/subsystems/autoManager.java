package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AutoManager{
    public enum autoDriveState{
        resting,
        intakeHandoff,
        shootHandoff,
        passing,
        spin;

        
        public Pose2d getGoal(){
            return new Pose2d();
        }

        public Command getDriveCommand(){
            return new Command(){};
        }

        public Command getHappyCommand(){
            return new Command(){};
        }

        

    }


    protected static autoDriveState state = autoDriveState.resting;
    protected static Command autoDriveCommand;

    public static void changeState(autoDriveState state){
        if (autoDriveCommand!=null){
            autoDriveCommand.cancel();
        }
        AutoManager.state=state;
    }

    public static void resting(){
        changeState(autoDriveState.resting);
    }

    public static Command startResting(){
        return new InstantCommand(AutoManager::resting);
    }

    public static void intake(){
        changeState(autoDriveState.intakeHandoff);
    }

    public static Command startIntake(){
        return new InstantCommand(AutoManager::intake);
    }

    public static void shooting(){
        changeState(autoDriveState.shootHandoff);
    }

    public static Command startShooting(){
        return new InstantCommand(AutoManager::shooting);
    }

    public static void passing(){
        changeState(autoDriveState.passing); 
    }
    
    public static Command startPassing(){
        return new InstantCommand(AutoManager::passing);
    }



    //One must imagine the autoManager happy.
    public boolean isHappy(){
        return autoDriveCommand==null&&autoDriveCommand.isScheduled();
    }

    public void periodic(){

    }

}