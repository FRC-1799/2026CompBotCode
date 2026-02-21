package frc.robot.commands.swervedrive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SystemManager;

public class spin extends Command{
    public spin(){
        addRequirements(SystemManager.swerve);
    }
    
    @Override
    public void execute(){
        SystemManager.swerve.drive(new Translation2d(), Math.PI, true);
    }
}
