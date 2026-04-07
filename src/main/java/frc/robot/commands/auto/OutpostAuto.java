package frc.robot.commands.auto;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.FieldPosits;
import frc.robot.SystemManager;
import frc.robot.Utils.utilFunctions;
import frc.robot.commands.AutoStates.ShootHandoff;
import frc.robot.subsystems.GeneralManager;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;

public class OutpostAuto extends SequentialCommandGroup{
    protected static boolean leftSide;
    public OutpostAuto(){
        super(

                       new DeferredCommand(
                ()->{
                    return new InstantCommand(
                        ()->{
                            leftSide = 
                            utilFunctions.getDistanceBetweenTwoPoints(SystemManager.getSwervePose(), FieldPosits.leftSideAutoHandoff).in(Meters) <
                            utilFunctions.getDistanceBetweenTwoPoints(SystemManager.getSwervePose(), FieldPosits.rightSideAutoHandoff).in(Meters);
                        }
                    );
                }, 
                Set.of()
            ),

            new InstantCommand(GeneralManager::startIntaking),

            new DeferredCommand(
                ()->{
                    return SystemManager.swerve.driveToPose(new Pose2d(FieldPosits.mid, new Rotation2d(
                        SystemManager.getSwervePose().getX()-FieldPosits.mid.getX(),
                        SystemManager.getSwervePose().getY()-FieldPosits.mid.getY()))
                    );
                },
                Set.of(SystemManager.swerve)
            ),

            new DeferredCommand(
                ()->{return SystemManager.swerve.driveToPose(leftSide? FieldPosits.leftSideAutoHandoff : FieldPosits.rightSideAutoHandoff, MetersPerSecond.of(2));},
                Set.of(SystemManager.swerve)
            ),            
            new ShootHandoff(),
  
            new InstantCommand(GeneralManager::startIntaking),

            SystemManager.swerve.driveToPose(FieldPosits.outpost),
            new ConditionalCommand(
                new InstantCommand(()->{
                    ((Arena2026Rebuilt)SimulatedArena.getInstance()).outpostDump(DriverStation.getAlliance().isPresent()&&DriverStation.getAlliance().get()!=Alliance.Blue);
                }),
                new InstantCommand(),
                RobotBase::isSimulation
            ),
                        
            new ShootHandoff()
        );
    }
}
