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

public class simIntake extends SubsystemBase{
    public IntakeSimulation intakeSim;
    

    public static enum intakeState{
        intaking,
        resting,
        backRun;

    }
    
    intakeState state = intakeState.resting;

   

    public simIntake(){
        if (RobotBase.isReal()){
            intakeSim= IntakeSimulation.InTheFrameIntake("Fuel", SystemManager.simButRealTrain, Meters.of(0.7), IntakeSimulation.IntakeSide.BACK, 40);
        }
        else{
            intakeSim= IntakeSimulation.InTheFrameIntake("Fuel", SystemManager.swerve.getMapleSimDrive().get(), Meters.of(0.7), IntakeSimulation.IntakeSide.BACK, 40);
        }

    }

    @Override 
    public void periodic(){
        


        if (state==intakeState.intaking){
            intakeSim.startIntake();
        }
        else{
            intakeSim.stopIntake();
        }
        

        SmartDashboard.putNumber("fuelCount", intakeSim.getGamePiecesAmount());
        SmartDashboard.putBoolean("intakeIsRunning", state==intakeState.intaking);


    }


    public int getPieceCount(){
        return intakeSim.getGamePiecesAmount();
    }


   

    public void stop(){
        state=intakeState.resting;
       
    }

    public void start(){
        state=intakeState.intaking;
    }

}