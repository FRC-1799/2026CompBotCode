package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SystemManager;

public abstract class elevatorIO extends SubsystemBase {

    protected double setpoint=Constants.elevatorConstants.startingPosit;

    protected Mechanism2d mech = new Mechanism2d(0,0);//Constants.elevatorConstants.fromRobotCenter.getX(), Constants.elevatorConstants.fromRobotCenter.getZ());//0.86, 1.75);
    protected MechanismRoot2d root = mech.getRoot("elevatorRoot", Constants.elevatorConstants.fromRobotCenter.getX(), Constants.elevatorConstants.fromRobotCenter.getY());
    protected MechanismLigament2d elevator = new MechanismLigament2d("elevator main", Constants.elevatorConstants.compressedLen, Constants.elevatorConstants.angle.getDegrees());

    public elevatorIO(){
        root.append(elevator);
        SmartDashboard.putData("elevator", mech);
    }

    /**
     * sets the setpoint of the elevator in raw encoder units. Use setSetpoint instead
     * @param setpoint the setpoint to set in raw encoder units.
     */
    public void setSetpointRaw(double setpoint){
        this.setpoint=setpoint;
    };

    /**
     * sets the setpoint of the elevator in meters
     * @param setpoint the height of the elevator in meters
     */
    public void setSetpoint(double setpoint){
        SmartDashboard.putNumber("most recent elevator update", setpoint);
        setSetpointRaw(setpoint);
    };

    /**@return wether or not the elevator is within tolerance of its setpoint*/
    public boolean isAtSetpoint(){
        return Math.abs(setpoint-getHeight())<Constants.elevatorConstants.tolerance;
    };

    /**@return the 3d translation from the bottom of the elevator to the current point. all measurements use the rotation point of the wrist for consistency*/
    public Translation3d getTranslation(){
        return new Translation3d(getHeightRender()*Math.cos(Constants.elevatorConstants.angle.getRadians()), 0, getHeightRender()*Math.sin(Constants.elevatorConstants.angle.getRadians())).plus(Constants.elevatorConstants.fromRobotCenter);
    };



    /**@return the height of the elevator in meters. all measurements use the rotation point of the wrist for consistency*/
    public double getHeight(){
        return this.getEncoderVal()/Constants.elevatorConstants.encoderToMeters;
    };
    public double getHeightRender(){
        return getHeight()+Constants.elevatorConstants.elevatorIntakeEndOffset;
    }

    /**resets the elevator to its starting config */
    public void reset(){
        setpoint=0;
    };

    public boolean isAtTop(){
        return Math.abs(getHeight()-Constants.elevatorConstants.maxHeight)<Constants.elevatorConstants.tolerance;

    }




    /**updates the internal mechanism  */
    public void updateRender(){
        
        elevator.setLength(getHeightRender()+Constants.elevatorConstants.fromRobotCenter.getZ());
    }

    /**@return returns the internal encoder value of the elevator encoder. use getHeight instead*/
    public abstract double getEncoderVal();
}
