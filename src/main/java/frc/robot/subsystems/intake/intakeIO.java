package frc.robot.subsystems.intake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SystemManager;
import frc.robot.Constants.elevatorConstants;

public abstract class intakeIO extends SubsystemBase{

    public enum hasPieceState{
		intaking,
		full,
		empty,
		starting	}

    public static enum intakeState{
        intaking,
        outtaking,
        resting,
        backRun;

    }

    protected intakeState state = intakeState.resting;
    BooleanSupplier stopTrigger=()->{return false;};


    /**sets the intake state to intaking until a piece is intaked */
    public void intake(){
        this.intakeUntil(()->this.hasPiece());
    }

    public void startBackrun(){
    }
    /**
     * sets the intake state to intake until trigger returns true 
     * @param trigger the supplier that will stop the intake when it returns true
    */
    public void intakeUntil(BooleanSupplier trigger){
        state=intakeState.intaking;
		stopTrigger=trigger;
    }

    /**sets the intake state to outtake until the piece is outtaked*/
    public void outtake(){
        this.outtakeUntil(()->!this.hasPiece());
    };

    /**
     * outtakes until the trigger returns true
     * @param trigger the supplier that will stop the intake when it returns true
     */
    public void outtakeUntil(BooleanSupplier trigger){
        state=intakeState.outtaking;
        stopTrigger=trigger;
    }


    /**resets the intake */
    public void reset(){
        stop();
		state = intakeState.resting;
    }

    /**@return the state of the intake in the form of a intakeState */
    public intakeState getState(){
        return state;
    }

    /**gets a translation that represents the change from the 0,0 of the intake the point a coral would be */
    public Translation3d getTranslation(){
        
        return new Translation3d(
            -(Math.sin(Constants.elevatorConstants.angle.getRadians())*Constants.intakeConstants.coralFromWristLen+Constants.intakeConstants.coralLength),
            0,
            Math.cos(Constants.elevatorConstants.angle.getRadians())*Constants.intakeConstants.coralFromWristLen)
            
            
            .plus(SystemManager.elevator.getTranslation());    
    }

    /**@return wether or not the intake currently contains a piece.*/
    public abstract boolean hasPiece();
    /**stops the intake */
    public abstract void stop();
   
} 
