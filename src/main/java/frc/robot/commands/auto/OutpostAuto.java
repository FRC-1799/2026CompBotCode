package frc.robot.commands.auto;

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
import frc.robot.commands.AutoStates.ShootHandoff;
import frc.robot.subsystems.GeneralManager;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;

public class OutpostAuto extends SequentialCommandGroup{
    public OutpostAuto(){
        super(

            GeneralManager.startIntaking(),
            SystemManager.swerve.driveToPose(FieldPosits.outpost),
            new ConditionalCommand(
                new InstantCommand(()->{
                    ((Arena2026Rebuilt)SimulatedArena.getInstance()).outpostDump(DriverStation.getAlliance().isPresent()&&DriverStation.getAlliance().get()!=Alliance.Blue);
                }),
                new InstantCommand(),
                RobotBase::isSimulation
            ),

            new ShootHandoff(),
            new WaitUntilCommand(()->SystemManager.intake.getPieceCount()==0),

                        
            GeneralManager.startIntaking(),

            new DeferredCommand(
                ()->{
                    return SystemManager.swerve.driveToPose(new Pose2d(FieldPosits.mid, new Rotation2d(
                        SystemManager.getSwervePose().getX()-FieldPosits.mid.getX(),
                        SystemManager.getSwervePose().getY()-FieldPosits.mid.getY()))
                    );
                },
                Set.of(SystemManager.swerve)
            ),
            new ShootHandoff()
        );
    }
}
