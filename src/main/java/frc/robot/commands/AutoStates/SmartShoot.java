package frc.robot.commands.AutoStates;

import java.util.function.BooleanSupplier;

import frc.robot.SystemManager;
import frc.robot.Constants.AutonConstants;
import frc.robot.subsystems.TimingManager;

public class SmartShoot extends ShootHandoff{
    public SmartShoot(){
        
        super(()->true);
        //super(SmartShoot::shouldShoot);
    }

    public SmartShoot(BooleanSupplier canHandoff){
        super (()->{return SmartShoot.shouldShoot()&&canHandoff.getAsBoolean();});
    }

    public static boolean shouldShoot(){

        return TimingManager.getInstance().isActive()==!(TimingManager.getInstance().timeRemaining()<AutonConstants.predictedShotTime);
    }
}
