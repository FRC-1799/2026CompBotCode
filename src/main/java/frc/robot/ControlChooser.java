package frc.robot;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Utils.BetterTrigger;
import frc.robot.Utils.DebugUtil;
import frc.robot.Utils.utilFunctions;
import frc.robot.commands.AutoStates.SmartShoot;
import frc.robot.commands.states.spitting;
import frc.robot.commands.swervedrive.AbsoluteFieldDrive;
import frc.robot.gamecontrollers.CommandPS5EdgeController;
import frc.robot.subsystems.GeneralManager;
import frc.robot.subsystems.GeneralManager.generalState;
import frc.robot.subsystems.Shooter.Shooter;

import java.util.function.Consumer;

import static edu.wpi.first.math.MathUtil.angleModulus;
import static edu.wpi.first.units.Units.*;
import static frc.robot.RobotPreferences.*;
import static frc.robot.gamecontrollers.ControllerUtil.getJoystickToHeading;

public class ControlChooser {

    RobotPreferences pref = RobotPreferences.getInstance();

    SendableChooser<EventLoop> chooser = new SendableChooser<>();
    Consumer<ControlChooser> current;

    CommandXboxController xbox1;
    CommandXboxController xbox2;
    CommandPS5EdgeController ps5_1;
    CommandPS5EdgeController ps5_2;

    EventLoop controlLoop = CommandScheduler.getInstance().getDefaultButtonLoop();


    /**
     * creates a control chooser
     */
    ControlChooser() {

        xbox1 = new CommandXboxController(Constants.controllerIDs.commandXboxController1ID);
        xbox2 = new CommandXboxController(Constants.controllerIDs.commandXboxController2ID);
        ps5_1 = new CommandPS5EdgeController(0);
        ps5_2 = new CommandPS5EdgeController(1);

        chooser.setDefaultOption("default", CommandScheduler.getInstance().getDefaultButtonLoop());

        if (!RobotBase.isReal()) {
            //for schemes too unsafe to run on the real bot
        }


        chooser.addOption("Field Oriented (can auto)", getFieldOrientedControl());
        chooser.addOption("Robot Oriented (no auto)", getRobotOrientedControl());
        chooser.addOption("PS5 Field Oriented", getPs5FieldOrientedControl());
        chooser.addOption("PS5 Robot Oriented", getPs5RobotOrientedControl());


        chooser.onChange((EventLoop scheme) -> {
            changeControl(scheme);
        });
        changeControl(chooser.getSelected());

        SmartDashboard.putData("Control chooser", chooser);

    }

    //returns an xbox controllers pov buttons in terms of degrees
    public static int getPOVForTest(CommandXboxController controller) {
        for (int pov : Constants.OperatorConstants.supportedPOV) {
            if (controller.pov(pov).getAsBoolean()) {
                return pov;
            }
        }
        return 0;

    }

    /**
     * configures a default command that can run on a loop.
     *
     * @param defaultCommand the command to make the default
     * @param subsystem      the subsystem this command is the default for
     * @param loop           the loop to attach the default command too
     */
    public static void setDefaultCommand(Command defaultCommand, Subsystem subsystem, EventLoop loop) {
        new BetterTrigger(loop, () -> ((CommandScheduler.getInstance().requiring(subsystem) == null || CommandScheduler.getInstance().requiring(subsystem) == defaultCommand))).whileTrue(defaultCommand);
    }

    /**
     * @param rawInput axis input -1 to 1
     * @param exponent 1=linear and above that the curve becomes more pronounced
     * @return
     */
    public static double powerCurve(double rawInput, double exponent) {
        double sign = Math.signum(rawInput);
        double absoluteValue = Math.abs(rawInput);
        return sign * Math.pow(absoluteValue, exponent);
    }

    /**
     * changes the control scheme to the scheme specified
     *
     * @param scheme the scheme to change too
     */
    public void changeControl(EventLoop scheme) {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().setActiveButtonLoop(scheme);

    }

    /**
     * restarts the control chooser
     */
    public void restart() {
        changeControl(chooser.getSelected());
    }

    /**
     * @return a new test control loop
     */
    private EventLoop getFieldOrientedControl() {
        EventLoop loop = new EventLoop();
        setDefaultCommand(
                new AbsoluteFieldDrive(
                        SystemManager.swerve,
                        () -> powerCurve(MathUtil.applyDeadband(-xbox1.getLeftY(), 0.1), pref.gameControllerAxisMag()),
                        () -> powerCurve(MathUtil.applyDeadband(-xbox1.getLeftX(), 0.1), pref.gameControllerAxisMag()),
                        () -> {
                            var overideHeading = getJoystickToHeading(-xbox1.getRightY(), xbox1.getRightX(), 0.1);
                            return overideHeading.orElseGet(() -> SystemManager.shooter.launchPrediiton.launcherYaw().getRadians());

                        })
                , SystemManager.swerve, loop);

        //xbox1.rightTrigger(0.4,loop).whileTrue(new IntakeHandoff()).onFalse(new InstantCommand(()->GeneralManager.cancelSpesificState(generalState.intaking)));
        //xbox1.leftTrigger(0.1,loop).whileTrue(new ShootHandoff(()->xbox1.getLeftTriggerAxis()>0.5)).onFalse(new InstantCommand(()->GeneralManager.cancelSpesificState(generalState.shooting)));
        xbox1.rightTrigger(0.4, loop).whileTrue(GeneralManager.intaking());
        xbox1.leftTrigger(0.4, loop).whileTrue(new SmartShoot());
        xbox1.a(loop).whileTrue(new SequentialCommandGroup(GeneralManager.shooting().until(() -> !SystemManager.shooter.hasPiecesRemaining())));
        //xbox1.x(loop).whileTrue(new SequentialCommandGroup(GeneralManager.shooting().until(() -> !SystemManager.shooter.hasPiecesRemaining())));

        //xbox1.leftTrigger(0.4, loop).whileTrue(new AimAtPoint(FieldPosits.hubPose2d));

        //xbox1.a(loop).whileTrue(GeneralManager.shooting());
        xbox1.b(loop).whileTrue(GeneralManager.spitting());

        return loop;
    }

    /**
     * @return a new test control loop
     */
    private EventLoop getRobotOrientedControl() {
        EventLoop loop = new EventLoop();
        setDefaultCommand(
                SystemManager.swerve.driveRobotOrientedCommand(
                        () -> powerCurve(MathUtil.applyDeadband(-xbox1.getLeftY(), 0.1), pref.gameControllerAxisMag()),
                        () -> powerCurve(MathUtil.applyDeadband(-xbox1.getRightX(), 0.1), pref.gameControllerAxisMag()),
                        () -> powerCurve(MathUtil.applyDeadband(-xbox1.getLeftX(), 0.1), pref.gameControllerAxisMag())
                ), SystemManager.swerve, loop);

        // xbox1.rightTrigger(0.4,loop).whileTrue(new IntakeHandoff()).onFalse(new InstantCommand(()->GeneralManager.cancelSpecificState(generalState.intaking)));
        // xbox1.leftTrigger(0.1,loop).whileTrue(new SmartShoot(()->xbox1.getLeftTriggerAxis()>0.5)).onFalse(new InstantCommand(()->GeneralManager.cancelSpecificState(generalState.shooting)));

        // xbox1.rightTrigger(0.4, loop).whileTrue(GeneralManager.intaking());

        //xbox1.leftTrigger(0.4, loop).whileTrue(new AimAtPoint(FieldPosits.hubPose2d));

        // xbox1.a(loop).whileTrue(GeneralManager.shooting());


        xbox1.rightTrigger(0.4, loop).whileTrue(GeneralManager.shooting());
        xbox1.leftTrigger(0.4, loop).toggleOnTrue(GeneralManager.intaking());
        xbox1.rightBumper(loop).whileTrue(new SmartShoot());
        xbox1.a(loop).toggleOnTrue(new spitting());


        return loop;
    }

    private boolean isIntaking() {
        return GeneralManager.state == generalState.intaking;
    }



    /**
     * @return a new test control loop
     */
    private EventLoop getPs5FieldOrientedControl() {
        EventLoop loop = new EventLoop();
        setDefaultCommand(
                new AbsoluteFieldDrive(
                        SystemManager.swerve,
                        () -> powerCurve(MathUtil.applyDeadband(-ps5_1.getLeftY(), 0.1), 4),
                        () -> powerCurve(MathUtil.applyDeadband(-ps5_1.getLeftX(), 0.1), 4),
                        () -> {
                            var overideHeading = getJoystickToHeading(-ps5_1.getRightY(), ps5_1.getRightX(), 0.1);
                            return overideHeading.orElseGet(() -> SystemManager.shooter.launchPrediiton.launcherYaw().getRadians());

                        })
                , SystemManager.swerve, loop);

        //xbox1.rightTrigger(0.4,loop).whileTrue(new IntakeHandoff()).onFalse(new InstantCommand(()->GeneralManager.cancelSpesificState(generalState.intaking)));
        //xbox1.leftTrigger(0.1,loop).whileTrue(new ShootHandoff(()->xbox1.getLeftTriggerAxis()>0.5)).onFalse(new InstantCommand(()->GeneralManager.cancelSpesificState(generalState.shooting)));
        ps5_1.R2(loop).whileTrue(GeneralManager.intaking());
        ps5_1.L2(loop).whileTrue(new SmartShoot());
        ps5_1.cross(loop).toggleOnTrue(new SmartShoot());
        ps5_1.square(loop).whileTrue(new SequentialCommandGroup(GeneralManager.shooting().until(() -> !SystemManager.shooter.hasPiecesRemaining())));

        //xbox1.leftTrigger(0.4, loop).whileTrue(new AimAtPoint(FieldPosits.hubPose2d));

        //xbox1.a(loop).whileTrue(GeneralManager.shooting());
        ps5_1.circle(loop).whileTrue(GeneralManager.spitting());

        return loop;
    }

    private EventLoop getPs5RobotOrientedControl() {

        EventLoop loop = new EventLoop();
        setDefaultCommand(
                SystemManager.swerve.driveRobotOrientedCommand(
                        () -> powerCurve(MathUtil.applyDeadband(-ps5_1.getLeftY(), 0.1), pref.gameControllerAxisMag()),
                        () -> powerCurve(MathUtil.applyDeadband(-ps5_1.getRightX(), 0.1), pref.gameControllerAxisMag()),
                        () -> powerCurve(MathUtil.applyDeadband(-ps5_1.getLeftX(), 0.1), pref.gameControllerAxisMag())
                ), SystemManager.swerve, loop);

        //ps5_1.rightTrigger(0.4,loop).whileTrue(new IntakeHandoff()).onFalse(new InstantCommand(()->GeneralManager.cancelSpesificState(generalState.intaking)));
        //ps5_1.leftTrigger(0.1,loop).whileTrue(new ShootHandoff(()->ps5_1.getLeftTriggerAxis()>0.5)).onFalse(new InstantCommand(()->GeneralManager.cancelSpesificState(generalState.shooting)));
        ps5_1.R2(loop).whileTrue(GeneralManager.intaking());
        ps5_1.L2(loop).whileTrue(new SequentialCommandGroup(GeneralManager.shooting().until(() -> !SystemManager.shooter.hasPiecesRemaining())));
        //ps5_1.cross(loop).toggleOnTrue(new SmartShoot());
        //ps5_1.square(loop).whileTrue(new SequentialCommandGroup(GeneralManager.shooting().until(() -> !SystemManager.shooter.hasPiecesRemaining())));


        //ps5_1.leftTrigger(0.4, loop).whileTrue(new AimAtPoint(FieldPosits.hubPose2d));

        //ps5_1.a(loop).whileTrue(GeneralManager.shooting());
        ps5_1.circle(loop).whileTrue(GeneralManager.spitting());


        return loop;
    }


}

