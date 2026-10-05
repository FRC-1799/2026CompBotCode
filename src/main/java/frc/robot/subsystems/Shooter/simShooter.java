package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.shooterConstants;
import frc.robot.RobotPreferences;
import frc.robot.SystemManager;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

public class simShooter extends Shooter {


    protected double cooldown = 0;
    protected double matchTime = DriverStation.getMatchTime();
    private RobotPreferences pref = RobotPreferences.getInstance();


    @Override
    public void periodic() {
        super.periodic();
        cooldown -= Timer.getFPGATimestamp() - matchTime;
        matchTime = Timer.getFPGATimestamp();
        if (state == shooterState.shooting) {
            if (cooldown <= 0) {
                shootInternal();
                cooldown = 0.2;
            }
        }
    }

    public void shootInternal() {

        if (SystemManager.intake.getPieceCount() > 0) {
            SystemManager.intake.removePiece();

            var launchResult = SystemManager
                    .calculateLaunchWorker
                    .getLaunchPrediction();

            if(launchResult.error() > 0.0) return;

            var launchVelocity = launchResult.launchVelocity();

            var robotPose = SystemManager.getRealPoseMaple().getTranslation();
            var shooterPoseRel = pref.shooterPose();

            SimulatedArena.getInstance()
                    .addGamePieceProjectile(new RebuiltFuelOnFly(
                            // Obtain robot position from drive simulation
                            robotPose,
                            // The scoring mechanism is installed at (0.46, 0) (meters) on the robot
                            shooterPoseRel.toPose2d().getTranslation(),
                            // Obtain robot speed from drive simulation
                            SystemManager.swerve.getFieldVelocity(),
                            // Obtain robot facing from drive simulation
                            SystemManager.getRealPoseMaple().getRotation(),
                            // The height at which the fuel is ejected
                            shooterPoseRel.getMeasureZ(),
                            // The initial speed of the fuel
                            //MetersPerSecond.of(getTopFlywheelSpeed().in(RPM) * shooterConstants.SimRPMToMPS),
                            launchVelocity,
                            // The fuel is ejected at a 35-degree slope
                            shooterConstants.shotAngle
                    ));
        }
    }

    @Override
    public boolean hasPiecesRemaining() {
        return SystemManager.intake.getPieceCount() != 0;
    }
}
