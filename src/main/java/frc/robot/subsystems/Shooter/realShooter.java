package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.shooterConstants;

public class realShooter extends Shooter{

    DigitalInput beamBreak = new DigitalInput(shooterConstants.beambreakPort);

    int cyclesWithoutBall=0;

    
    @Override
    public void periodic(){
        super.periodic();

        if (beamBreak.get()){
            cyclesWithoutBall=0;
        }
        else{
            cyclesWithoutBall++;
        }

        SmartDashboard.putBoolean("Shooter/hasBalls", hasPiecesRemaining());


    }

    @Override
    public boolean hasPiecesRemaining(){
        return shooterConstants.shouldBeambreak && cyclesWithoutBall>shooterConstants.countsWithoutBallToBeEmpty;
    }
}