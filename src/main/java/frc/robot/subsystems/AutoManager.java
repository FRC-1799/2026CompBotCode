package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.AutonConstants;
import frc.robot.FieldPosits;
import frc.robot.SystemManager;
import frc.robot.Utils.utilFunctions;
import frc.robot.commands.auto.smallAutoDrive;
import frc.robot.commands.auto.spin;


/**An additionally state manager to handle fancier states with autoDrive. */
public class AutoManager{
    public enum autoDriveState{
        resting(null, null, ()->false),

        intakeHandoff(
            new ConditionalCommand(
                SystemManager.swerve.driveToPose(SystemManager.getSwervePose().nearest(FieldPosits.intakingHandoffPoses), AutonConstants.intakeHandoffSpeed),
                new InstantCommand(),
                ()->FieldPosits.alianceZone.contains(SystemManager.getSwervePose().getTranslation())
            ),
            GeneralManager.startIntaking()       
        ),
        shootHandoff(
            new ConditionalCommand(
                SystemManager.swerve.driveToPose(SystemManager.getSwervePose().nearest(FieldPosits.scoringPoses)),
                new InstantCommand(),
                ()->!FieldPosits.alianceZone.contains(SystemManager.getSwervePose().getTranslation())
            ),
            GeneralManager.startShooting()  
        ),

        passing(
            new smallAutoDrive(
                new Pose2d(
                    SystemManager.getSwervePose().getTranslation(),
                    utilFunctions.getAngleBetweenTwoPoints(SystemManager.getSwervePose(), SystemManager.getSwervePose().nearest(FieldPosits.passingPoses))
                )
            ),
            GeneralManager.startPassing()

        ),

        spin(
            new spin(),
            new InstantCommand(),
            ()->false
        );

        protected Command driveCommand;
        protected Command happyCommand;
        protected BooleanSupplier happySupplier; 

        private autoDriveState(Command driveCommand, Command happyCommand, BooleanSupplier happySupplier){
            this.driveCommand=driveCommand;
            this.happyCommand=happyCommand;
        }

        private autoDriveState(Command driveCommand, Command happyCommand){
            this(driveCommand, happyCommand, SystemManager::swerveIsAtGoal);
        }

        public Command getDriveCommand(){
            return driveCommand;
        }

        public Command getHappyCommand(){
            return happyCommand;
        }

        public boolean isHappy(){
            return happySupplier.getAsBoolean();
        }

        

    }


    protected static autoDriveState state = autoDriveState.resting;
    protected static boolean isHappy =false;

    public static void changeState(autoDriveState state){
        if (state.getDriveCommand()!=null){
            state.getDriveCommand().cancel();
        }

        AutoManager.state=state;
        if (AutoManager.state!=null) AutoManager.state.driveCommand.schedule();
        isHappy=false;
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

    public static void spin(){
        changeState(autoDriveState.spin);
    }

    public static Command startSpin(){
        return new InstantCommand(AutoManager::spin);
    }



    //One must imagine the autoManager happy.
    public static boolean isHappy(){
        return isHappy;
    }

    public static void periodic(){
        if (state!=autoDriveState.resting)
            if (!isHappy && state.isHappy()){
                isHappy=true;
                if (state.getHappyCommand()!=null)state.getHappyCommand().schedule();
                resting();
            }
    }

    

}