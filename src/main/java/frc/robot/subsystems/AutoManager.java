package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Set;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.AutonConstants;
import frc.robot.FieldPosits;
import frc.robot.SystemManager;
import frc.robot.Utils.utilFunctions;
import frc.robot.commands.swervedrive.smallAutoDrive;
import frc.robot.commands.swervedrive.spin;


/**An additionally state manager to handle fancier states with autoDrive. */
public class AutoManager{
    public enum autoDriveState{
        resting(null, null, ()->false),

        intakeHandoff(
            new DeferredCommand(
                ()->{
                    return new ConditionalCommand(
                        SystemManager.swerve.driveToPose(SystemManager.getSwervePose().nearest(FieldPosits.intakingHandoffPoses), AutonConstants.intakeHandoffSpeed),
                        new InstantCommand(()->{SystemManager.autoDriveGoal=SystemManager.getSwervePose();}),
                        ()->FieldPosits.alianceZone.contains(SystemManager.getSwervePose().getTranslation())
                    );
                }
            ,new HashSet<Subsystem>()),
            GeneralManager.startIntaking()
        ),
        shootHandoff(
            new DeferredCommand(
                ()->{return new ConditionalCommand(
                    SystemManager.swerve.driveToPose(new Pose2d(
                        SystemManager.getSwervePose().nearest(FieldPosits.scoringPoses).getTranslation(),
                        utilFunctions.getAngleBetweenTwoPoints(
                            new Pose2d(SystemManager.getSwervePose().nearest(FieldPosits.scoringPoses).getTranslation(), new Rotation2d()),
                            FieldPosits.hubPose2d)

                    )),
                     SystemManager.swerve.driveToPose(new Pose2d(SystemManager.getSwervePose().getTranslation(), utilFunctions.getAngleBetweenTwoPoints(new Pose2d(SystemManager.getSwervePose().getTranslation(), new Rotation2d()), FieldPosits.hubPose2d))),
                    ()->!FieldPosits.alianceZone.contains(SystemManager.getSwervePose().getTranslation())
                );}
            ,new HashSet<Subsystem>()),
            GeneralManager.startShooting(), 
            ()->{return 
                SystemManager.swerveIsAtGoal()&&
                utilFunctions.pythagorean(SystemManager.swerve.getFieldVelocity().vxMetersPerSecond, SystemManager.swerve.getFieldVelocity().vxMetersPerSecond)<0.05&&
                Math.abs(SystemManager.swerve.getFieldVelocity().omegaRadiansPerSecond)<0.02;
            }
        ),

        passing(
            new DeferredCommand(
                ()->{return new smallAutoDrive(
                    new Pose2d(
                        SystemManager.getSwervePose().getTranslation(),
                        utilFunctions.getAngleBetweenTwoPoints(SystemManager.getSwervePose(), SystemManager.getSwervePose().nearest(FieldPosits.passingPoses))
                    )
                    );
                },
                new HashSet<Subsystem>()
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
            this.happySupplier=happySupplier;
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
            
            return happySupplier!=null&& happySupplier.getAsBoolean();
        }

        

    }


    protected static autoDriveState state = autoDriveState.resting;

    public static void changeState(autoDriveState state){
        if (state.getDriveCommand()!=null){
            state.getDriveCommand().cancel();
        }

        AutoManager.state=state;
        if (AutoManager.state.getDriveCommand()!=null) AutoManager.state.getDriveCommand().schedule();
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

    public static void periodic(){
        SmartDashboard.putString("AutoDrive/State", state.toString());
        SmartDashboard.putBoolean("AutoDrive/Drive is active", state.getDriveCommand()!=null&&CommandScheduler.getInstance().isScheduled(state.getDriveCommand()));
        SmartDashboard.putString("AutoDrive/CurrentDrive", SystemManager.swerve.getCurrentCommand()!=null?SystemManager.swerve.getCurrentCommand().getName() : "NULL");
        if (state!=autoDriveState.resting)
            if (state.isHappy()){
                if (state.getHappyCommand()!=null)state.getHappyCommand().schedule();
                resting();
            }
    }

    

}