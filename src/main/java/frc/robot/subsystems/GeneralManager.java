package frc.robot.subsystems;

import java.util.HashSet;
import java.util.Set;

import frc.robot.SystemManager;
import frc.robot.commands.states.*;

import edu.wpi.first.util.function.BooleanConsumer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class GeneralManager{

    /**enum to represent all the available states */
    public enum generalState{
        resting(new resting()),
        shooting(new shooting()),
        spitting(new spitting()),
        intaking(new intaking()),
        passing(new passing());

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
      resting();
      subsystems.add(SystemManager.intake);
      subsystems.add(SystemManager.shooter);

    }


    /**should be called periodically to keep the general manager up to date */
    public static void periodic(){
        if (state!=null&&!CommandScheduler.getInstance().isScheduled(state.state)){
            state=null;
        }

        if (state==null){
            resting();
        }
        
        SmartDashboard.putString("general state", state.state.getName());
        

    }



    /**<h2>Changes the current state to the resting state</h2>*/
    public static void resting(){
        startState(generalState.resting);
    }

    /**
     * <h2>Creates a command that will start the resting state when called</h2>
     * @return A command too start resting state
     */
    public static InstantCommand startResting(){
        return new InstantCommand(()->{resting();});
    }

    /**<h2>Changes the current state to the shooting state</h2>*/
    public static void shooting(){
        startState(generalState.shooting);
    }

    /**
     * <h2>Creates a command that will start the shooting state when called</h2>
     * @return A command too start resting state
     */
    public static InstantCommand startShooting(){
                return new InstantCommand(()->{shooting();});
    }

    /**<h2>Changes the current state to the spitting state</h2>*/
    public static void spitting(){
        startState(generalState.spitting);
    }

    /**
     * <h2>Creates a command that will start the spitting state when called</h2>
     * @return A command too start resting state
     */
    public static InstantCommand startSpitting(){
                return new InstantCommand(()->{spitting();});
    }

    /**<h2>Changes the current state to the intaking state</h2>*/
    public static void intaking(){
        startState(generalState.intaking);
    }

    /**
     * <h2>Creates a command that will start the intaking state when called</h2>
     * @return A command too start resting state
     */
    public static InstantCommand startIntaking(){
                return new InstantCommand(()->{intaking();});
    }

    /**<h2>Changes the current state to the passing state</h2>*/
    public static void passing(){
        startState(generalState.passing);
    }

    /**
     * <h2>Creates a command that will start the passing state when called</h2>
     * @return A command too start resting state
     */
    public static InstantCommand startPassing(){
                return new InstantCommand(()->{passing();});
    }



    /**
     * starts the provided state
     * @param state the state to start
     */
    public static void startState(generalState state){
        if (GeneralManager.state!= null){
            CommandScheduler.getInstance().cancel(GeneralManager.state.state);
        }
        GeneralManager.state=state;
    
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


    /**sets an internal callback that will be used ONCE the next time a state is finished */
    public static void setExternalEndCallback(BooleanConsumer callback){
        externalCallback=callback;
    }
}
