package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radian;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SystemManager;

public class simShooter extends SubsystemBase{
    public enum shooterState{
        resting, 
        rev,
        shooting
    }
    
        private static final String Inches = null;
    
        protected double cooldown=0;
    
        public shooterState state = shooterState.resting;
        public simShooter(){
    
        }
    
        @Override
        public void periodic(){
            cooldown--;
            if (state==shooterState.shooting){
                if (cooldown<=0){
                    shootInternal();
                    cooldown=10;
                }
            }
        }
    
    
    public void shootInternal(){
            
        if (SystemManager.intake.getPieceCount()>0){
            SystemManager.intake.intakeSim.obtainGamePieceFromIntake();
            
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
                // The height at which the coral is ejected
                Inch.of(21),
            // The initial speed of the coral
            MetersPerSecond.of(8),
            // The coral is ejected at a 35-degree slope
            Degrees.of(70)));
            

        }
    }

    public void startShooting(){
        state=shooterState.shooting;
    }

    public void stop(){
        state = shooterState.resting;
    }
    
}
