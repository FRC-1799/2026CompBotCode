package frc.robot.commands.auto;



import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.SystemManager;
import frc.robot.Utils.scoringPosit;
import frc.robot.Utils.utilFunctions;
import frc.robot.subsystems.autoManager;
import frc.robot.subsystems.generalManager;


public class ScorePiece extends Command{
    scoringPosit posit;
    boolean mechIsFinished=false;
    Command driveCommand;
    Command mechCommand;
    boolean hasSpit;
    boolean driveIsFinished;
    protected StructPublisher<Pose2d> goalPublisher = NetworkTableInstance.getDefault().getStructTopic("ScorePieceGoal", Pose2d.struct).publish(PubSubOption.keepDuplicates(true));


    /**
     * scores a piece at the defined scoring posit
     * @param posit the posit to score
     */
    public ScorePiece(scoringPosit posit){
        this.posit=posit;
        
    }


    /**initializes the command */
    @Override
    public void initialize(){
        //sets the mechanisms to the proper score state
        generalManager.scoreAt(posit.level.getAsInt());

        //starts auto drive
        driveCommand=SystemManager.swerve.driveToPose(posit.getScorePose());
        driveCommand.schedule();
        System.out.println(posit.level.getAsInt());

        //gets the mech command and sets the proper mech callback
        mechCommand=generalManager.getStateCommand();
        generalManager.setExternalEndCallback(this::mechIsFinishedCall);
        goalPublisher.set(posit.getScorePose());
        

        //reinitializes state booleans used
        mechIsFinished=false;
        hasSpit=false;
        driveIsFinished=false;
    }

    
     /**called ever rio cycle while the command is scheduled*/
    @Override
    public void execute() {
        //restarts the drive command if it finished early
        if (!driveCommand.isScheduled()){
            if (utilFunctions.pythagorean(SystemManager.getSwervePose().getX(), posit.getScorePose().getX(), SystemManager.getSwervePose().getY(), posit.getScorePose().getY())
                >=Constants.AutonConstants.autoDriveScoreTolerance){
                if (
                    utilFunctions.pythagorean(SystemManager.getSwervePose().getX(), posit.getScorePose().getX(),
                    SystemManager.getSwervePose().getY(), posit.getScorePose().getY())
                    <=Constants.AutonConstants.distanceWithinPathplannerDontWork){

                    driveCommand= new smallAutoDrive(posit.getScorePose());
                }
                driveCommand.schedule();
            }
            else{
                driveIsFinished=true;
            }
        }

        SmartDashboard.putBoolean("DriveIsFinished", driveIsFinished);

        //starts outtake if relevant
        if (mechIsFinished&&driveIsFinished){
            generalManager.outtake();
            generalManager.setExternalEndCallback(this::intakeIsFinishedCall);
        }

        //debug info
        SmartDashboard.putNumber("reef pole", posit.pole.getRowAsIndex());
    }
   
     /**
     * function to be called when the mech is in its proper state
     * @param wasInterrupted wether or not the intake routine was canceled
     */
    public void mechIsFinishedCall(boolean wasInterrupted){
        if (wasInterrupted){
            cancel();
        }

        mechIsFinished=true;
    }

    /**
     * function to be called when the outtake has happened
     * @param wasInterrupted wether or not the intake routine was canceled
     */
    public void intakeIsFinishedCall(boolean wasInterrupted){
        hasSpit=true;
    }

    /**
     * @return true once the piece has been outtaked
     */
    @Override
    public boolean isFinished(){
        return hasSpit;
    }
    

    /**
     * command called when the command finishes
     * @param wasInterrupted wether or not the command was canceled
    */
    @Override
    public void end(boolean wasInterrupted){
        if (driveCommand.isScheduled()){
            driveCommand.cancel();
            
        }
        if (!wasInterrupted){
            autoManager.cycleCount++;
            autoManager.score+=posit.getPointValForItem();
        }
        
    }   
}
