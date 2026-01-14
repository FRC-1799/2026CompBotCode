package frc.robot;


import java.util.function.Consumer;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Utils.BetterTrigger;
import frc.robot.Utils.utilFunctions;

import frc.robot.commands.swervedrive.AbsoluteDriveAdv;
import frc.robot.commands.swervedrive.AbsoluteFieldDrive;
import frc.robot.subsystems.autoManager;
import frc.robot.subsystems.swervedrive.AimAtPoint;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;

public class ControlChooser {

    SendableChooser<EventLoop> chooser=new SendableChooser<>();
    Consumer<ControlChooser>current;
    
    CommandXboxController xbox1;
    CommandXboxController xbox2;
    
    EventLoop controlLoop=CommandScheduler.getInstance().getDefaultButtonLoop();
    

    
    /**creates a control chooser */
    ControlChooser(){
        
        xbox1=new CommandXboxController(Constants.controllerIDs.commandXboxController1ID);
        xbox2=new CommandXboxController(Constants.controllerIDs.commandXboxController2ID);

        chooser.setDefaultOption("default", CommandScheduler.getInstance().getDefaultButtonLoop());

        if (!RobotBase.isReal()){
            //for schemes too unsafe to run on the real bot
        }


        chooser.addOption("testControl", getTestControl());
        chooser.addOption("runAutoControl", runAutoDrive());
        chooser.addOption("lameControl", lameControl());

        
        
        chooser.onChange((EventLoop scheme)->{changeControl(scheme);});
        changeControl(chooser.getSelected());
        
        SmartDashboard.putData("Control chooser", chooser);
       
    }


    /**
     * changes the control scheme to the scheme specified
     * @param scheme the scheme to change too
     */
    public void changeControl(EventLoop scheme){
        CommandScheduler.getInstance().cancelAll();
        autoManager.takeControl();
        CommandScheduler.getInstance().setActiveButtonLoop(scheme);

    }

    /**restarts the control chooser */
    public void restart(){
        changeControl(chooser.getSelected());
    }

    

    //returns an xbox controllers pov buttons in terms of degrees
    public static int getPOVForTest(CommandXboxController controller){
        for (int pov: Constants.OperatorConstants.supportedPOV){
            if (controller.pov(pov).getAsBoolean()){
                return pov;
            }
        } 
        return 0;

    }

    /**
     * configures a default command that can run on a loop.
     * @param defaultCommand the command to make the default
     * @param subsystem the subsystem this command is the default for
     * @param loop the loop to attach the default command too
     */
    public static void setDefaultCommand(Command defaultCommand, Subsystem subsystem, EventLoop loop){
        new BetterTrigger(loop, ()->((CommandScheduler.getInstance().requiring(subsystem)==null||CommandScheduler.getInstance().requiring(subsystem)==defaultCommand))).whileTrue(defaultCommand);
    }


    /**@return a new test control loop*/
    private EventLoop getTestControl(){
        EventLoop loop = new EventLoop();
        setDefaultCommand(new AbsoluteFieldDrive(SystemManager.swerve, ()->-xbox1.getLeftY(), ()->-xbox1.getLeftX(), ()->{
            if(utilFunctions.pythagorean(xbox1.getRightX(), xbox1.getRightY())>=0.2)return Math.atan2(-xbox1.getRightX(), -xbox1.getRightY())/Math.PI; return SystemManager.swerve.getHeading().getRadians()/Math.PI;})
           ,SystemManager.swerve, loop);
            
        xbox1.rightTrigger(0.4,loop).onTrue(new InstantCommand(SystemManager.intake::start)).onFalse(new InstantCommand(SystemManager.intake::stop));
        xbox1.leftTrigger(0.4,loop).onTrue(new InstantCommand(SystemManager.shooter::startShooting)).onFalse(new InstantCommand(SystemManager.shooter::stop));
        xbox1.leftTrigger(0.4, loop).whileTrue(new AimAtPoint(FieldPosits.blueHubPose));
        
        xbox1.a(loop).onTrue(new InstantCommand(()->((Arena2026Rebuilt)SimulatedArena.getInstance()).outpostDump(false)));
        xbox1.b(loop).onTrue(new InstantCommand(()->((Arena2026Rebuilt)SimulatedArena.getInstance()).outpostThrowForGoal(false)));



        return loop;
    }





    private EventLoop runAutoDrive(){
        EventLoop loop = new EventLoop();

    

        new Trigger(loop, ()->SystemManager.robot.heartBeat%2==1).onTrue(new InstantCommand(()->autoManager.giveControl()));

       
        return loop;
    }

    private EventLoop lameControl(){
        EventLoop loop = new EventLoop();
        setDefaultCommand(new AbsoluteDriveAdv(SystemManager.swerve, ()->-xbox1.getLeftY(), ()->-xbox1.getLeftX(), ()->-xbox1.getLeftTriggerAxis()+xbox1.getRightTriggerAxis(), xbox1.pov(180), xbox1.pov(0), xbox1.pov(90), xbox1.pov(270))
           ,SystemManager.swerve, loop);

        return loop;
    }



}

