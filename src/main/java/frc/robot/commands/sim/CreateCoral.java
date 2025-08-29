package frc.robot.commands.sim;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.FieldPosits;

public class CreateCoral extends InstantCommand{
    public Pose2d dropSpot;

    /**
     * 
     * @param posit one of the following; leftLeft, leftMid, leftRight, rightLeft, rightMid, rightRight
     * @throws Error if posit is not one of the above options
     */
    public CreateCoral(String posit){
        switch (posit){
            case "leftLeft":
                dropSpot=FieldPosits.IntakePoints.leftLeft;
                break;
            case "leftMid":
                dropSpot=FieldPosits.IntakePoints.leftMid;
                break;
            case "leftRight":
                dropSpot=FieldPosits.IntakePoints.leftRight;
                break;
            case "rightLeft":
                dropSpot=FieldPosits.IntakePoints.rightLeft;
                break;
            case "rightMid":
                dropSpot=FieldPosits.IntakePoints.rightMid;
                break;
            case "rightRight":
                dropSpot=FieldPosits.IntakePoints.rightRight;
                break;
            default:
                throw new Error("Tried to instantiate a coral creator with an invalid string " + posit);
        }
    }

    /**
     * constructs a command that makes a coral at the entered pose2d
     * @param posit the pose to spawn the coral
     */
    public CreateCoral(Pose2d posit){
        dropSpot=posit;
    }

    /**runs once every time the command is scheduled */
    @Override
    public void execute(){
        SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralOnField(dropSpot));
    }
}
