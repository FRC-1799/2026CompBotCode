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
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

public class simIntake extends Intake{
    public IntakeSimulation intakeSim;
    

    
    protected intakeState state = intakeState.resting;

    protected double backRunState=0;

   

    public simIntake(){
        if (RobotBase.isReal()){
            intakeSim= IntakeSimulation.InTheFrameIntake("Fuel", SystemManager.simButRealTrain, Meters.of(0.7), IntakeSimulation.IntakeSide.BACK, 40);
        }
        else{
            intakeSim= IntakeSimulation.InTheFrameIntake("Fuel", SystemManager.swerve.getMapleSimDrive().get(), Meters.of(0.7), IntakeSimulation.IntakeSide.BACK, 40);
        }

    }


    public int getPieceCount(){
        return intakeSim.getGamePiecesAmount();
    }


    protected void intake(){
        intakeSim.startIntake();
    }

    protected void shutDown(){
        intakeSim.stopIntake();
    }

    public void removePiece(){
        intakeSim.obtainGamePieceFromIntake();
    }


    @Override
    protected void backRun() {
        if (backRunState==0&&getPieceCount()!=0){
            removePiece();
            
            SimulatedArena.getInstance()
                .addGamePieceProjectile(new RebuiltFuelOnFly(
                    // Obtain robot position from drive simulation
                    SystemManager.getRealPoseMaple().getTranslation(),
                    // The scoring mechanism is installed at (0.46, 0) (meters) on the robot
                    new Translation2d(),
                    // Obtain robot speed from drive simulation
                    SystemManager.swerve.getFieldVelocity(),
                    // Obtain robot facing from drive simulation
                    SystemManager.getRealPoseMaple().getRotation(),
                    // The height at which the fuel is ejected
                    Meters.of(0),
                // The initial speed of the fuel
                MetersPerSecond.of(2),
                // The fuel is ejected at a 35-degree slope
                Degrees.of(0)
                ));

                backRunState=5;
        }
        else{
            backRunState--;
        }
            
    }

}