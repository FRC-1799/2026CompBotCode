package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.AutonConstants;
import frc.robot.FieldPosits;
import frc.robot.SystemManager;
import frc.robot.Utils.utilFunctions;
import frc.robot.commands.AutoStates.IntakeHandoff;
import frc.robot.commands.AutoStates.PassingHandoff;
import frc.robot.commands.AutoStates.SemiAutoState;
import frc.robot.commands.AutoStates.ShootHandoff;
import frc.robot.commands.swervedrive.smallAutoDrive;
import frc.robot.commands.swervedrive.spin;


/**An additionally state manager to handle fancier states with autoDrive. */
public class AutoManager{
    public enum autoDriveState{
        resting(new SemiAutoState(new RunCommand(()->{}))),

        intakeHandoff(new IntakeHandoff()),
        shootHandoff(new ShootHandoff()),
        passing(new PassingHandoff()),
        spin(new SemiAutoState(new spin()));

        protected SemiAutoState state;

        private autoDriveState(SemiAutoState stateSupplier){
            this.state = stateSupplier;
        }

        public SemiAutoState getStateCommand(){
            return state;
        }

        public boolean isScheduled(){
            return state.isScheduled();
        }

        

    }


    protected static autoDriveState state = autoDriveState.resting;

    public static void changeState(autoDriveState state){
        AutoManager.state.state.cancel();

        AutoManager.state=state;
        state.state.schedule();
    }

    public static void resting(){
        changeState(autoDriveState.resting);
    }

    public static Command startResting(){
        return new SequentialCommandGroup(new InstantCommand(AutoManager::resting), new WaitUntilCommand(()->{return state!=autoDriveState.resting;}));
    }

    public static void intake(){
        changeState(autoDriveState.intakeHandoff);
    }

    public static Command startIntake(){
        return new SequentialCommandGroup(new InstantCommand(AutoManager::intake), new WaitUntilCommand(()->{return state!=autoDriveState.intakeHandoff;}));
    }

    public static void shooting(){
        changeState(autoDriveState.shootHandoff);
    }

    public static Command startShooting(){
        return new SequentialCommandGroup(new InstantCommand(AutoManager::shooting), new WaitUntilCommand(()->{return state!=autoDriveState.shootHandoff;}));
    }

    public static void passing(){
        changeState(autoDriveState.passing); 
    }
    
    public static Command startPassing(){
        return new SequentialCommandGroup(new InstantCommand(AutoManager::passing), new WaitUntilCommand(()->{return state!=autoDriveState.passing;}));
    }

    public static void spin(){
        changeState(autoDriveState.spin);
    }

    public static Command startSpin(){
        return new SequentialCommandGroup(new InstantCommand(AutoManager::spin), new WaitUntilCommand(()->{return state!=autoDriveState.spin;}));
    }

    public static void periodic(){
        SmartDashboard.putString("AutoDrive/State", state.toString());
        SmartDashboard.putBoolean("AutoDrive/Drive is active", state.isScheduled());
        SmartDashboard.putString("AutoDrive/CurrentDrive", SystemManager.swerve.getCurrentCommand()!=null?SystemManager.swerve.getCurrentCommand().getName() : "NULL");
        if (state!=autoDriveState.resting)
            if (!state.isScheduled()){
                resting();
            }
    }

    

}