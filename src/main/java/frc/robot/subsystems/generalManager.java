package frc.robot.subsystems;

import java.util.HashSet;
import java.util.Set;

import frc.robot.SystemManager;
import frc.robot.FieldPosits.reefLevel;
import frc.robot.FieldPosits.reefLevel.algaeRemovalEnum;
import frc.robot.Utils.warningManager;
import frc.robot.commands.states.*;

import edu.wpi.first.util.function.BooleanConsumer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class generalManager{

    /**enum to represent all the available states */
    public enum generalState{
        intake(new intaking()),
        start(new starting()),
        L1(new scoreConfig(reefLevel.L1)),
        L2(new scoreConfig(reefLevel.L2)),
        L3(new scoreConfig(reefLevel.L3)),
        L4(new scoreConfig(reefLevel.L4)),
        algaeLow( new removeAlgaeConfig(algaeRemovalEnum.low)),
        algaeHigh(new removeAlgaeConfig(algaeRemovalEnum.high)),
        algaeRemoval(new removeAlgaeAct()),
        outtake(new outtaking()),
        resting(new resting());

        Command state;

        /**
         * creates a state with the given command
         * @param command the command to execute when the state is scheduled
         */
        private generalState(Command command){
            state=command;
        }

        /**def function so states can start being implemented before their command is complete */
        private generalState(){
            throw new Error("Attempted to start a state that has not been implemented");
        }
    }


    public static generalState state;
    public static BooleanConsumer externalCallback=null;
    public static Set<Subsystem> subsystems = new HashSet<>();
    
    
    
    /**initializes the general manager. Should be called before any other general manager actions are taken*/
    public static void generalManagerInit(){
      start();
      subsystems.add(SystemManager.intake);
      subsystems.add(SystemManager.elevator);
      subsystems.add(SystemManager.algaeRemover);
    }


    /**should be called periodically to keep the general manager up to date */
    public static void periodic(){
        if (state!=null&&!CommandScheduler.getInstance().isScheduled(state.state)){
            warningManager.throwAlert(warningManager.badGeneralRoutine);
            state=null;
        }

        if (state==null){
            resting();
        }
        
        SmartDashboard.putString("general state", state.state.getName());
        

    }


    /**
     * changes the state to a scoring config at the given level
     * @param level the level to score at
     */
    public static void scoreAt(int level){
        switch (level) {
            case 0:
                scoreL1();
                break;
            case 1:
                scoreL2();
                break;
            case 2:
                scoreL3();
                break;
            case 3:
                scoreL4();
                break;
            default:
                throw new Error("Attempted to score at an invalid level");
        }
    }


    /**changes the current state to score l1 */
    public static void scoreL1(){
        startState(generalState.L1);
    }

    /**changes the current state to score l2 */
    public static void scoreL2(){
        startState(generalState.L2);
    }
    
    /**changes the current state to score l3 */
    public static void scoreL3(){
        startState(generalState.L3);
    }

    /**changes the current state to score l4 */
    public static void scoreL4(){
        startState(generalState.L4);
    }


    public static void algaeConfigHigh(){
        startState(generalState.algaeHigh);
    }

    public static void algaeConfigLow(){
        startState(generalState.algaeLow);
    }

    public static void algaeConfig(boolean isLow){
        if(isLow){
            algaeConfigLow();
        }
        else{
            algaeConfigHigh();
        }
    }

    public static void algaeRemove(){
        startState(generalState.algaeRemoval);
    }

    /**changes the current state to the intaking state */
    public static void intake(){
        startState(generalState.intake);
    }

    /**changes the current state to the outtaking state */
    public static void outtake(){
        startState(generalState.outtake);
    }

    /**changes the current state to the resting state */
    public static void resting(){
        startState(generalState.resting);
    }

    /**changes the current state to the starting state */
    public static void start(){
        startState(generalState.start);
    }

    /**
     * starts the provided state
     * @param state the state to start
     */
    public static void startState(generalState state){
        generalManager.state=state;
        // System.out.println(state.state.getName());
        // System.out.println("\n\n\n\n\n");
        CommandScheduler.getInstance().schedule(state.state);
    }

    /**
     * @return the command managing the current state
     */
    public static Command getStateCommand(){
        return state.state;
    }

    /**
     * @return the current state as a general state
     */
    public static generalState getState(){
        return state;
    }

    /**
     * to be called whenever a state command finishes
     * @param wasInterrupted whether or not the command was interrupted
     */
    public static void endCallback(boolean wasInterrupted){
        
        if (externalCallback!=null){
            externalCallback.accept(wasInterrupted);
            externalCallback=null;
        }
    }

    /**@return wether or not the current state is a scoring config type state */
    public static boolean isScoringState() {
        return state==generalState.L1 || state==generalState.L2 || state==generalState.L3 || state==generalState.L4;
    }

    /**sets an internal callback that will be used ONCE the next time a state is finished */
    public static void setExternalEndCallback(BooleanConsumer callback){
        externalCallback=callback;
    }
}
