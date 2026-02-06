package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.shooterConstants;

public class realShooter extends Shooter{
    protected TalonFX indexer = new TalonFX(shooterConstants.indexerID);

    @Override
    public void periodic(){
        super.periodic();
        if (state==shooterState.shooting){
            indexer.set(shooterConstants.indexerSpeed);
        }
        else indexer.set(0);
    }
}
