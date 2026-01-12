
package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.utils.FieldMirroringUtils;


/**it just works, not important code so dont worry about it */
public class AIRobotInSimulation implements Subsystem {
    /* if an opponent robot is not requested to be on field, it queens outside the field for performance */
    public static final Pose2d[] ROBOT_QUEENING_POSITIONS = new Pose2d[] {
        new Pose2d(-6, 0, new Rotation2d()),
        new Pose2d(-5, 0, new Rotation2d()),
        new Pose2d(-4, 0, new Rotation2d()),
        new Pose2d(-3, 0, new Rotation2d()),
        new Pose2d(-2, 0, new Rotation2d())
    };
    
    public static final Pose2d[] ROBOTS_STARTING_POSITIONS = new Pose2d[] {
        new Pose2d(15, 6, Rotation2d.fromDegrees(180)),
        new Pose2d(15, 4, Rotation2d.fromDegrees(180)),
        new Pose2d(15, 2, Rotation2d.fromDegrees(180)),
        new Pose2d(1.6, 6, new Rotation2d()),
        new Pose2d(1.6, 4, new Rotation2d())
    };

    public static final Twist2d backLeft = new Twist2d(-0.5, -0.5, 0);
    public static final Twist2d frontRight= new Twist2d(0.5, 0.5, 0);

    public static final AIRobotInSimulation[] instances = new AIRobotInSimulation[5];

    private static final DriveTrainSimulationConfig AI_ROBOT_CONFIG =
            DriveTrainSimulationConfig.Default().withRobotMass(Kilograms.of(45));

    private static final RobotConfig robotConfig = new RobotConfig(
            55,
            8,
            new ModuleConfig(
                    Units.inchesToMeters(2), 3.5, 1.2, DCMotor.getFalcon500(1).withReduction(8.14), 60, 1),
            0.6);
    private static final PPHolonomicDriveController driveController =
            new PPHolonomicDriveController(new PIDConstants(5.0, 0.02), new PIDConstants(7.0, 0.05));



    /**initializes a opponent robot on the first available slot */
    public static void startOpponentRobotSimulations() {
        try {
            instances[0] = new AIRobotInSimulation(
                    PathPlannerPath.fromPathFile("Opponent robot cycle path 0"),
                    Commands.none(),
                    PathPlannerPath.fromPathFile("Opponent robot cycle path 0 backwards"),
                    Commands.none(),
                    ROBOT_QUEENING_POSITIONS[0],
                    1);
        } catch (Exception e) {
            DriverStation.reportError("failed to load opponent robot simulation path, error:" + e.getMessage(), false);
        }
    }

    public static AIRobotInSimulation getRobotAtIndex(int index){
        return instances[index];
    }

    

    public final SelfControlledSwerveDriveSimulation driveSimulation;
    private final int id;

    public AIRobotInSimulation(
            PathPlannerPath segment0,
            Command toRunAtEndOfSegment0,
            PathPlannerPath segment1,
            Command toRunAtEndOfSegment1,
            Pose2d queeningPose,
            int id) {
        this.id = id;
        this.driveSimulation =
                new SelfControlledSwerveDriveSimulation(new SwerveDriveSimulation(AI_ROBOT_CONFIG, queeningPose));
        SendableChooser<Command> behaviorChooser = new SendableChooser<>();
        behaviorChooser.setDefaultOption(
                "None", Commands.run(() -> driveSimulation.setSimulationWorldPose(queeningPose), this));
        behaviorChooser.addOption(
                "Auto Cycle", getAutoCycleCommand(segment0, toRunAtEndOfSegment0, segment1, toRunAtEndOfSegment1));
        behaviorChooser.addOption("Joystick Drive", getJoystickDriveCommand());
        behaviorChooser.onChange((Command::schedule));
        RobotModeTriggers.teleop()
                .onTrue(Commands.runOnce(() -> behaviorChooser.getSelected().schedule()));
        RobotModeTriggers.disabled()
                .onTrue(Commands.runOnce(
                                () -> {
                                    driveSimulation.setSimulationWorldPose(queeningPose);
                                    driveSimulation.runChassisSpeeds(
                                            new ChassisSpeeds(), new Translation2d(), false, false);
                                },
                                this)
                        .ignoringDisable(true));

        SmartDashboard.putData("AIRobotBehaviors/Opponent Robot " + id + " Behavior", behaviorChooser);
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation.getDriveTrainSimulation());
    }

    private Command getAutoCycleCommand(
            PathPlannerPath segment0,
            Command toRunAtEndOfSegment0,
            PathPlannerPath segment1,
            Command toRunAtEndOfSegment1) {
        final SequentialCommandGroup cycle = new SequentialCommandGroup();
        final Pose2d startingPose = new Pose2d(
                segment0.getStartingDifferentialPose().getTranslation(),
                segment0.getIdealStartingState().rotation());
        cycle.addCommands(
                opponentRobotFollowPath(segment0).andThen(toRunAtEndOfSegment0).withTimeout(10));
        cycle.addCommands(
                opponentRobotFollowPath(segment1).andThen(toRunAtEndOfSegment1).withTimeout(10));

        return cycle.repeatedly()
                .beforeStarting(Commands.runOnce(() -> driveSimulation.setSimulationWorldPose(
                        FieldMirroringUtils.toCurrentAlliancePose(startingPose))));
    }



    private Command opponentRobotFollowPath(PathPlannerPath path) {
        return new FollowPathCommand(
                path,
                driveSimulation::getActualPoseInSimulationWorld,
                driveSimulation::getActualSpeedsRobotRelative,
                (speeds, feedForwards) -> driveSimulation.runChassisSpeeds(speeds, new Translation2d(), false, false),
                driveController,
                robotConfig,
                FieldMirroringUtils::isSidePresentedAsRed,
                this);
    }

    private Command getJoystickDriveCommand() {

        final XboxController joystick = new XboxController(id);
        final Supplier<ChassisSpeeds> joystickSpeeds = () -> new ChassisSpeeds(
            -joystick.getLeftY() * driveSimulation.maxLinearVelocity().in(MetersPerSecond), // Forward/Backward
            -joystick.getLeftX() * driveSimulation.maxLinearVelocity().in(MetersPerSecond), // Left/Right
            -joystick.getRightX() * driveSimulation.maxAngularVelocity().in(RadiansPerSecond) // Rotation
            );

        return Commands.run(
                        () -> {
                            // Calculate field-centric speed from the driver station-centric speed
                            final ChassisSpeeds fieldCentricSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                                    joystickSpeeds.get(),
                                    FieldMirroringUtils.getCurrentAllianceDriverStationFacing()
                                            .plus(Rotation2d.fromDegrees(180)));
                            // Run the field-centric speed to control the robot's movement
                            driveSimulation.runChassisSpeeds(fieldCentricSpeeds, new Translation2d(), true, true);
                        },
                        this)
                // Before the command starts, reset the robot to its starting position on the field
                .beforeStarting(() -> driveSimulation.setSimulationWorldPose(
                        FieldMirroringUtils.toCurrentAlliancePose(ROBOTS_STARTING_POSITIONS[id - 1])));
    }

    public static Pose2d[] getOpponentRobotPoses() {
        return getRobotPoses(new AIRobotInSimulation[] {instances[0], instances[1], instances[2]});
    }

    public static Pose2d[] getAlliancePartnerRobotPoses() {
        return getRobotPoses(new AIRobotInSimulation[] {instances[3], instances[4]});
    }

    public static Pose2d[] getRobotPoses(AIRobotInSimulation[] instances) {
        return Arrays.stream(instances)
                .map(instance -> instance.driveSimulation.getActualPoseInSimulationWorld())
                .toArray(Pose2d[]::new);
    }

    private Translation2d getTopCorner(){
        return driveSimulation.getActualPoseInSimulationWorld().exp(frontRight).getTranslation();
    }

    private Translation2d getBottomCorner(){
        return driveSimulation.getActualPoseInSimulationWorld().exp(backLeft).getTranslation();
    }
    


    public Translation2d applyVelocitySmear(Translation2d corner){
        Translation2d velocityTrans = getVelocityOver(0.1);
        Translation2d finalTrans = getVelocityOver(0.25);
        Translation2d newCorner=corner.plus(velocityTrans);
        if (newCorner.getDistance(driveSimulation.getActualPoseInSimulationWorld().getTranslation())>corner.getDistance(driveSimulation.getActualPoseInSimulationWorld().getTranslation())){
            return corner.plus(finalTrans);
        }
        return corner;

    }

    public Pair<Translation2d, Translation2d> getHitbox(){
        return Pair.of(getTopCorner(), getBottomCorner());
    }

    public List<Pair<Translation2d, Translation2d>> getTrajectoryHitboxes(){
        List<Pair<Translation2d, Translation2d>> hitboxes = new ArrayList<>();
        Pair<Translation2d, Translation2d> CurrentHitbox = getHitbox();
        hitboxes.add(CurrentHitbox); 
        Translation2d  velocityTrans = getVelocityOver(0.25);
        for(double i = 0.05; i<0.25; i+=0.05){

            hitboxes.add(Pair.of(CurrentHitbox.getFirst().plus(velocityTrans.times(i)), CurrentHitbox.getSecond().plus(velocityTrans.times(i))));
        }
        return hitboxes;
    }

    public Translation2d getVelocity(){
        return getVelocityOver(1);
    }
    public Translation2d getVelocityOver(double seconds){
        return new Translation2d(driveSimulation.getActualSpeedsFieldRelative().vxMetersPerSecond*seconds, driveSimulation.getActualSpeedsFieldRelative().vyMetersPerSecond*seconds);
    }

}



 
// package frc.robot.subsystems.swervedrive;

// import static edu.wpi.first.units.Units.Kilograms;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.commands.FollowPathCommand;
// import com.pathplanner.lib.config.ModuleConfig;
// import com.pathplanner.lib.config.PIDConstants;
// import com.pathplanner.lib.config.RobotConfig;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;
// import com.pathplanner.lib.path.GoalEndState;
// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.path.Waypoint;
// import com.pathplanner.lib.pathfinding.Pathfinder;

// import edu.wpi.first.math.Pair;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.geometry.Twist2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.Subsystem;
// import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
// import frc.robot.FieldPosits;

// import java.util.ArrayList;
// import java.util.Arrays;
// import java.util.List;
// import java.util.function.Supplier;
// import org.ironmaple.simulation.SimulatedArena;
// import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
// import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
// import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
// import org.ironmaple.utils.FieldMirroringUtils;


// /**it just works, not important code so dont worry about it */
// public class AIRobotInSimulation implements Subsystem {
//     /* if an opponent robot is not requested to be on field, it queens outside the field for performance */
//     public static final Pose2d[] ROBOT_QUEENING_POSITIONS = new Pose2d[] {
//         new Pose2d(-6, 0, new Rotation2d()),
//         new Pose2d(-5, 0, new Rotation2d()),
//         new Pose2d(-4, 0, new Rotation2d()),
//         new Pose2d(-3, 0, new Rotation2d()),
//         new Pose2d(-2, 0, new Rotation2d())
//     };
    
//     public static final Pose2d[] ROBOTS_STARTING_POSITIONS = new Pose2d[] {
//         new Pose2d(15, 6, Rotation2d.fromDegrees(180)),
//         new Pose2d(15, 4, Rotation2d.fromDegrees(180)),
//         new Pose2d(15, 2, Rotation2d.fromDegrees(180)),
//         new Pose2d(1.6, 6, new Rotation2d()),
//         new Pose2d(1.6, 4, new Rotation2d())
//     };

//     public static final Twist2d backLeft = new Twist2d(-0.5, -0.5, 0);
//     public static final Twist2d frontRight= new Twist2d(0.5, 0.5, 0);

//     public static final AIRobotInSimulation[] instances = new AIRobotInSimulation[5];

//     private static final DriveTrainSimulationConfig AI_ROBOT_CONFIG =
//             DriveTrainSimulationConfig.Default().withRobotMass(Kilograms.of(45));

//     private static final RobotConfig robotConfig = new RobotConfig(
//             55,
//             8,
//             new ModuleConfig(
//                     Units.inchesToMeters(2), 3.5, 1.2, DCMotor.getFalcon500(1).withReduction(8.14), 60, 1),
//             0.6);
//     private static final PPHolonomicDriveController driveController =
//             new PPHolonomicDriveController(new PIDConstants(5.0, 0.02), new PIDConstants(7.0, 0.05));



//     /**initializes a opponent robot on the first available slot */
//     public static void startOpponentRobotSimulations() {
//         try {
//             instances[0] = new AIRobotInSimulation(
//                     PathPlannerPath.fromPathFile("opponent robot cycle path 0"),
//                     Commands.none(),
//                     PathPlannerPath.fromPathFile("opponent robot cycle path 0 backwards"),
//                     Commands.none(),
//                     ROBOT_QUEENING_POSITIONS[0],
//                     1);
//         } catch (Exception e) {
//             DriverStation.reportError("failed to load opponent robot simulation path, error:" + e.getMessage(), false);
//         }
//     }

//     public static AIRobotInSimulation getRobotAtIndex(int index){
//         return instances[index];
//     }

    

//     public final SelfControlledSwerveDriveSimulation driveSimulation;
//     private final int id;

//     public AIRobotInSimulation(
//             PathPlannerPath segment0,
//             Command toRunAtEndOfSegment0,
//             PathPlannerPath segment1,
//             Command toRunAtEndOfSegment1,
//             Pose2d queeningPose,
//             int id) {
//         this.id = id;
        
//         this.driveSimulation =
//                 new SelfControlledSwerveDriveSimulation(new SwerveDriveSimulation(AI_ROBOT_CONFIG, queeningPose));
//         SendableChooser<Command> behaviorChooser = new SendableChooser<>();
//         behaviorChooser.setDefaultOption(
//                 "None", Commands.run(() -> driveSimulation.setSimulationWorldPose(queeningPose), this));
//         behaviorChooser.addOption(
//                 "Auto Cycle", getAutoCycleCommand(segment0, toRunAtEndOfSegment0, segment1, toRunAtEndOfSegment1));
//         behaviorChooser.addOption("Joystick Drive", getJoystickDriveCommand());

        
//         behaviorChooser.addOption("randomChoice", opponentRobotFollowPath(new PathPlannerPath(getRandomPath(), PathConstraints.unlimitedConstraints(12), null, new GoalEndState(0, new Rotation2d(0))))
//                         .beforeStarting(() -> driveSimulation.setSimulationWorldPose(FieldMirroringUtils.toCurrentAlliancePose(ROBOTS_STARTING_POSITIONS[id - 1]))).repeatedly());
//         behaviorChooser.onChange((Command::schedule));
//         RobotModeTriggers.teleop()
//                 .onTrue(Commands.runOnce(() -> behaviorChooser.getSelected().schedule()));
//         RobotModeTriggers.disabled()
//                 .onTrue(Commands.runOnce(
//                                 () -> {
//                                     driveSimulation.setSimulationWorldPose(queeningPose);
//                                     driveSimulation.runChassisSpeeds(
//                                             new ChassisSpeeds(), new Translation2d(), false, false);
//                                 },
//                                 this)
//                         .ignoringDisable(true));

//         SmartDashboard.putData("AIRobotBehaviors/Opponent Robot " + id + " Behavior", behaviorChooser);
//         SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation.getDriveTrainSimulation());
//     }

//     private Command getAutoCycleCommand(
//             PathPlannerPath segment0,
//             Command toRunAtEndOfSegment0,
//             PathPlannerPath segment1,
//             Command toRunAtEndOfSegment1) {
//         final SequentialCommandGroup cycle = new SequentialCommandGroup();
//         final Pose2d startingPose = new Pose2d(
//                 segment0.getStartingDifferentialPose().getTranslation(),
//                 segment0.getIdealStartingState().rotation());
//         cycle.addCommands(
//                 opponentRobotFollowPath(segment0).andThen(toRunAtEndOfSegment0).withTimeout(10));
//         cycle.addCommands(
//                 opponentRobotFollowPath(segment1).andThen(toRunAtEndOfSegment1).withTimeout(10));

//         return cycle.repeatedly();
//                 // .beforeStarting(Commands.runOnce(() -> driveSimulation.setSimulationWorldPose(
//                 //         FieldMirroringUtils.toCurrentAlliancePose(startingPose))));
//     }

//     private List<Waypoint> getRandomPath(){
//         return PathPlannerPath.waypointsFromPoses(
//             FieldPosits.scoringPosits.scoringPoles[(int)Math.floor(Math.random()*12)].getScorePosit(), FieldPosits.IntakePoints.coralSpawnPoints[(int)Math.floor(Math.random()*6)]
//     );
//     }





//     private Command opponentRobotFollowPath(PathPlannerPath path) {
//         return new FollowPathCommand(
//                 path,
//                 driveSimulation::getActualPoseInSimulationWorld,
//                 driveSimulation::getActualSpeedsRobotRelative,
//                 (speeds, feedForwards) -> driveSimulation.runChassisSpeeds(speeds, new Translation2d(), false, false),
//                 driveController,
//                 robotConfig,
//                 ()->DriverStation.getAlliance().get()==DriverStation.Alliance.Blue,
//                 this);
//     }

//     private Command getJoystickDriveCommand() {
//         final XboxController joystick = new XboxController(id-1);
//         final Supplier<ChassisSpeeds> joystickSpeeds = () -> new ChassisSpeeds(
//                 -joystick.getLeftY() * 3.5, -joystick.getLeftX() * 3.5, -joystick.getRightX() * Math.toRadians(360));
//         final Supplier<Rotation2d> opponentDriverStationFacing = () ->
//                 FieldMirroringUtils.getCurrentAllianceDriverStationFacing().plus(Rotation2d.fromDegrees(180));
//         return Commands.run(
//                         () -> {
//                             final ChassisSpeeds fieldCentricSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
//                                     joystickSpeeds.get(),
//                                     FieldMirroringUtils.getCurrentAllianceDriverStationFacing()
//                                             .plus(Rotation2d.fromDegrees(180)));
//                             driveSimulation.runChassisSpeeds(joystickSpeeds.get(), new Translation2d(), true, true);
                            
//                         },
//                         this)
//                 .beforeStarting(() -> driveSimulation.setSimulationWorldPose(
//                         FieldMirroringUtils.toCurrentAlliancePose(ROBOTS_STARTING_POSITIONS[id - 1])));
//     }

//     public static Pose2d[] getOpponentRobotPoses() {
//         return getRobotPoses(new AIRobotInSimulation[] {instances[0], instances[1], instances[2]});
//     }

//     public static Pose2d[] getAlliancePartnerRobotPoses() {
//         return getRobotPoses(new AIRobotInSimulation[] {instances[3], instances[4]});
//     }

//     public static Pose2d[] getRobotPoses(AIRobotInSimulation[] instances) {
//         return Arrays.stream(instances)
//                 .map(instance -> instance.driveSimulation.getActualPoseInSimulationWorld())
//                 .toArray(Pose2d[]::new);
//     }

//     private Translation2d getTopCorner(){
//         return driveSimulation.getActualPoseInSimulationWorld().exp(frontRight).getTranslation();
//     }

//     private Translation2d getBottomCorner(){
//         return driveSimulation.getActualPoseInSimulationWorld().exp(backLeft).getTranslation();
//     }
    


//     public Translation2d applyVelocitySmear(Translation2d corner){
//         Translation2d velocityTrans = getVelocityOver(0.1);
//         Translation2d finalTrans = getVelocityOver(0.25);
//         Translation2d newCorner=corner.plus(velocityTrans);
//         if (newCorner.getDistance(driveSimulation.getActualPoseInSimulationWorld().getTranslation())>corner.getDistance(driveSimulation.getActualPoseInSimulationWorld().getTranslation())){
//             return corner.plus(finalTrans);
//         }
//         return corner;

//     }

//     public Pair<Translation2d, Translation2d> getHitbox(){
//         return Pair.of(getTopCorner(), getBottomCorner());
//     }

//     public List<Pair<Translation2d, Translation2d>> getTrajectoryHitboxes(){
//         List<Pair<Translation2d, Translation2d>> hitboxes = new ArrayList<>();
//         Pair<Translation2d, Translation2d> CurrentHitbox = getHitbox();
//         hitboxes.add(CurrentHitbox); 
//         Translation2d  velocityTrans = getVelocityOver(0.25);
//         for(double i = 0.05; i<0.25; i+=0.05){
//             //Pair<Translation2d, Translation2d> trajectoryHitbox = CurrentHitbox;
//             //trajectoryHitbox.= trajectoryHitbox.first.plus(velocityTrans.times(i));
//             hitboxes.add(Pair.of(CurrentHitbox.getFirst().plus(velocityTrans.times(i)), CurrentHitbox.getSecond().plus(velocityTrans.times(i))));
//         }
//         return hitboxes;
//     }

//     public Translation2d getVelocity(){
//         return getVelocityOver(1);
//     }
//     public Translation2d getVelocityOver(double seconds){
//         return new Translation2d(driveSimulation.getActualSpeedsFieldRelative().vxMetersPerSecond*seconds, driveSimulation.getActualSpeedsFieldRelative().vyMetersPerSecond*seconds);
//     }

// }
