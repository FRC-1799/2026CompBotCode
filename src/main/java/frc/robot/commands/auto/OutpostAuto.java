package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.FieldPosits;
import frc.robot.SystemManager;
import frc.robot.commands.AutoStates.ShootHandoff;
import frc.robot.subsystems.GeneralManager;

public class OutpostAuto extends SequentialCommandGroup{
    public OutpostAuto(){
        super(
            GeneralManager.startIntaking(),

            SystemManager.swerve.driveToPose(FieldPosits.outpost),
            new ShootHandoff(),

            new WaitUntilCommand(()->SystemManager.intake.getPieceCount()==0),

                        
            GeneralManager.startIntaking(),
            SystemManager.swerve.driveToPose(new Pose2d(FieldPosits.mid, new Rotation2d(
                SystemManager.getSwervePose().getX()-FieldPosits.mid.getX(),
                SystemManager.getSwervePose().getY()-FieldPosits.mid.getY()))
            ),
            new ShootHandoff()
        );
    }
}
