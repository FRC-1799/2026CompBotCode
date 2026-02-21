package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radian;


import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SystemManager;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation;

public abstract class Intake extends SubsystemBase{
    

    public static enum intakeState{
        intaking,
        resting,
        backRun;

    }
    
    intakeState state = intakeState.resting;

   



    @Override 
    public void periodic(){
        


        if (state==intakeState.intaking){
            intake();
        }
        else if (state==intakeState.backRun){
            backRun();
        }
        else{
            shutDown();
        }
        

        SmartDashboard.putNumber("fuelCount", getPieceCount());
        SmartDashboard.putBoolean("intakeIsRunning", state==intakeState.intaking);


    }


    public abstract int getPieceCount();


   

    public void stop(){
        state=intakeState.resting;
       
    }

    public void start(){
        state=intakeState.intaking;
    }

    public void outtake(){
        state=intakeState.backRun;
    }

    protected abstract void intake();

    protected abstract void shutDown();

    protected abstract void backRun();

    /**Removes an game piece from a simulated intake if aplicable. This method will do nothing on a real intake*/
    public void removePiece(){
        return;
    }

}