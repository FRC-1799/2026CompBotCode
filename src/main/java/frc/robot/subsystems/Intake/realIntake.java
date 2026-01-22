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
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SystemManager;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class realIntake extends SubsystemBase{
    public SparkFlex intakeMotor;
    

    public static enum intakeState{
        intaking,
        resting,
        backRun;

    }
    
    intakeState state = intakeState.resting;

   

    public realIntake(){
        intakeMotor = new SparkFlex(8, MotorType.kBrushless);
    }

    @Override 
    public void periodic(){
        


        if (state==intakeState.intaking){
            this.intake();
        }
        else if (state == intakeState.backRun) {
            this.outtake();
        }
        else {
            this.stop();
        }
        

        // SmartDashboard.putNumber("fuelCount", intakeSim.getGamePiecesAmount());
        SmartDashboard.putBoolean("intakeIsRunning", state==intakeState.intaking);


    }


    // public int getPieceCount(){
    //     return intakeMotor.getGamePiecesAmount();
    // }

    public void intake() {
        intakeMotor.set(0.01);
    }

    public void outtake() {
        intakeMotor.set(0.01);
    }
   

    public void stop(){
        state=intakeState.resting;
       
    }

    public void start(){
        state=intakeState.intaking;
    }

}