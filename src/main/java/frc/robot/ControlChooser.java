package frc.robot;


import java.util.HashSet;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.FieldPosits.reefLevel;
import frc.robot.FieldPosits.reefPole;
import frc.robot.Utils.BetterTrigger;
import frc.robot.Utils.scoringPosit;
import frc.robot.Utils.utilFunctions;
import frc.robot.commands.auto.IntakePieceCommand;
import frc.robot.commands.auto.ScorePiece;
import frc.robot.commands.auto.smallAutoDrive;
import frc.robot.commands.swervedrive.AbsoluteDriveAdv;
import frc.robot.commands.swervedrive.AbsoluteFieldDrive;
import frc.robot.subsystems.autoManager;
import frc.robot.subsystems.generalManager;

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


        chooser.addOption("autoTestControl", getAutoTestControl());
        chooser.addOption("testControl", getTestControl());
        chooser.addOption("StandardXboxControl", standardXboxControl());
        chooser.addOption("demoControl", demoControl());
        chooser.addOption("runAutoControl", runAutoDrive());
        chooser.addOption("autoAlign", autoAlignControl());
        chooser.addOption("stinkyControl", stinkyControl());
        chooser.addOption("SysIDControll", sysIDControl());
        
        
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
       
       xbox1.y(loop).onTrue(new InstantCommand(()->generalManager.scoreL4()));
       xbox1.x(loop).onTrue(new InstantCommand(()->generalManager.scoreL3()));
       xbox1.b(loop).onTrue(new InstantCommand(()->generalManager.scoreL2()));
       xbox1.a(loop).onTrue(new InstantCommand(()->generalManager.scoreL1()));
       xbox1.leftBumper(loop).onTrue(new InstantCommand(()->generalManager.algaeConfig(false)));
       xbox1.rightBumper(loop).onTrue(new InstantCommand(()->generalManager.algaeConfig(true)));

       xbox1.rightTrigger(0.4,loop).onTrue(new InstantCommand(()->generalManager.intake()));
       xbox1.rightStick(loop).onTrue(new InstantCommand(()->generalManager.outtake()));
       xbox1.leftTrigger(0.4, loop).onTrue(new InstantCommand(()->generalManager.algaeRemove()));

        return loop;
    }

    private EventLoop autoAlignControl(){
        EventLoop loop = new EventLoop();

        setDefaultCommand(new AbsoluteDriveAdv(SystemManager.swerve, ()->-xbox1.getLeftY(), ()->-xbox1.getLeftX(), ()->xbox1.getRightX(), xbox1.pov(180), xbox1.pov(0), xbox1.pov(90), xbox1.pov(270))
        ,SystemManager.swerve, loop);
        
        xbox2.a(loop).whileTrue(new DeferredCommand(()->new ScorePiece(new scoringPosit(reefLevel.CreateFromLevel(SystemManager.compass.getLevel()), reefPole.fromInt(SystemManager.compass.getPole()))), new HashSet<Subsystem>()));
        xbox2.rightBumper(loop).whileTrue(new DeferredCommand(()->new IntakePieceCommand(FieldPosits.IntakePoints.coralSpawnPoints[SystemManager.compass.getSlot()+3]), new HashSet<Subsystem>()));
        xbox2.leftBumper(loop).whileTrue(new DeferredCommand(()->new IntakePieceCommand(FieldPosits.IntakePoints.coralSpawnPoints[SystemManager.compass.getSlot()]), new HashSet<Subsystem>()));

        return loop;
    }





    

    /**@return a new standardXboxControl loop */
    private EventLoop standardXboxControl(){
        EventLoop loop = new EventLoop();
        setDefaultCommand(new AbsoluteFieldDrive(SystemManager.swerve, ()->-xbox1.getLeftY(), ()->-xbox1.getLeftX(), ()->{
            if(utilFunctions.pythagorean(xbox1.getRightX(), xbox1.getRightY())>=0.2)return Math.atan2(-xbox1.getRightX(), -xbox1.getRightY())/Math.PI; return SystemManager.swerve.getHeading().getRadians()/Math.PI;})
            ,SystemManager.swerve, loop);
        //setDefaultCommand(SystemManager.swerve.driveCommand(()->0, ()->0, ()->xbox1.getLeftX(), ()->xbox1.getLeftY()), SystemManager.swerve, loop);
        xbox1.b(loop).onTrue(SystemManager.swerve.driveToPose(new Pose2d(10,10, new Rotation2d(Math.PI))));
        xbox1.a(loop).onTrue(SystemManager.swerve.driveToPose(Constants.driveConstants.startingPosit));
        xbox1.y(loop).onTrue(new smallAutoDrive(Constants.driveConstants.startingPosit));

        return loop;
    }

    /**@return a new demo control loop */
    private EventLoop demoControl(){
        EventLoop loop  = new EventLoop();
        setDefaultCommand(new AbsoluteFieldDrive(SystemManager.swerve, ()->xbox1.getLeftX(), ()->-xbox1.getLeftY(),()-> getPOVForTest(xbox1)),SystemManager.swerve, loop);
        ///xbox1.b(loop).onTrue(SystemManager.swerve.driveToPose(new Pose2d(15,1.2, new Rotation2d(Math.PI))));
        return loop;
    }

    private EventLoop sysIDControl(){
        EventLoop loop = new EventLoop();

        new Trigger(loop, ()->{return xbox1.getLeftTriggerAxis()>=0.6&&xbox1.getRightTriggerAxis()>=0.6;}).onTrue(SystemManager.elevator.sysId());
        return loop;
    }

    /**@return a new auto test control loop */
    private EventLoop getAutoTestControl(){
        EventLoop loop = new EventLoop();

        setDefaultCommand(new AbsoluteFieldDrive(SystemManager.swerve, ()->-xbox1.getLeftY(), ()->-xbox1.getLeftX(), ()->{
            if(utilFunctions.pythagorean(xbox1.getRightX(), xbox1.getRightY())>=0.2)return Math.atan2(-xbox1.getRightX(), -xbox1.getRightY())/Math.PI; return SystemManager.swerve.getHeading().getRadians()/Math.PI;})
           ,SystemManager.swerve, loop);
        new Trigger(loop, xbox1.leftTrigger(0.75)).onTrue(new InstantCommand(()->autoManager.giveControl())).onFalse(new InstantCommand(()->autoManager.takeControl()));
        
        xbox1.b(loop).whileTrue(SystemManager.swerve.driveToPose(Constants.driveConstants.startingPosit));
        xbox1.y(loop).whileTrue(new smallAutoDrive(Constants.driveConstants.startingPosit));
        xbox1.x(loop).onTrue(new InstantCommand(()->SystemManager.reefIndexer.resetSIMONLY()));
       
        return loop;
    }

    private EventLoop runAutoDrive(){
        EventLoop loop = new EventLoop();

    

        new Trigger(loop, ()->SystemManager.robot.heartBeat%2==1).onTrue(new InstantCommand(()->autoManager.giveControl()));

       
        return loop;
    }

    private EventLoop stinkyControl(){
        EventLoop loop = new EventLoop();
        setDefaultCommand(new AbsoluteDriveAdv(SystemManager.swerve, ()->-xbox1.getLeftY(), ()->-xbox1.getLeftX(), ()->-xbox1.getLeftTriggerAxis()+xbox1.getRightTriggerAxis(), xbox1.pov(180), xbox1.pov(0), xbox1.pov(90), xbox1.pov(270))
           ,SystemManager.swerve, loop);
       

       
       xbox2.y(loop).onTrue(new InstantCommand(()->generalManager.scoreL4()));
       xbox2.x(loop).onTrue(new InstantCommand(()->generalManager.scoreL3()));
       xbox2.b(loop).onTrue(new InstantCommand(()->generalManager.scoreL2()));
       xbox2.a(loop).onTrue(new InstantCommand(()->generalManager.scoreL1()));
       xbox2.leftStick(loop).onTrue(new InstantCommand(()->generalManager.algaeConfig(false)));
       xbox2.rightStick(loop).onTrue(new InstantCommand(()->generalManager.algaeConfig(true)));

       xbox2.rightTrigger(0.4,loop).onTrue(new InstantCommand(()->generalManager.intake()));
       xbox2.rightBumper(loop).onTrue(new InstantCommand(()->generalManager.outtake()));
       xbox2.leftTrigger(0.4, loop).onTrue(new InstantCommand(()->generalManager.algaeRemove()));

        return loop;
    }



}

