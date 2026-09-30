package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.shooterConstants;
import frc.robot.FieldPosits;
import frc.robot.RobotPreferences;
import frc.robot.SystemManager;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.shooterConstants.topMotorConstants.WHEEL_DIAMETER;
import static frc.robot.subsystems.Shooter.ProjectileCalculatorCommon.calculateLinearVelocity;
import static frc.robot.subsystems.Shooter.ProjectileCalculatorExt.calculateLaunch;

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

            var wheelRpm = calculateLinearVelocity(calculateTopSpinnerRpm(), WHEEL_DIAMETER);
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
                            wheelRpm,
                            // The fuel is ejected at a 35-degree slope
                            shooterConstants.shotAngle
                    ));


        }
    }

    @Override
    public boolean hasPiecesRemaining() {
        return SystemManager.intake.getPieceCount() != 0;
    }

    public AngularVelocity calculateTopSpinnerRpm() {
        var rpmPref = RobotPreferences.getInstance().shootingSpeedRPM();

        AngularVelocity launchAngularVelocity;

        if (rpmPref <= 1) {
            var robotVelocity = SystemManager.swerve.getFieldVelocity();
            var robotPose3d = new Pose3d(SystemManager.getRealPoseMaple());
            var shooterPoseRel = pref.shooterPose();
            var pitchAngle = shooterPoseRel.getRotation().getMeasureY();
            var targetPoseRel = pref.targetRelativePose();

            var shooterPoseTransform = new Transform3d(shooterPoseRel.getTranslation(), shooterPoseRel.getRotation());
            var targetPoseTransform = new Transform3d(targetPoseRel.getTranslation(), targetPoseRel.getRotation());

            var shooterPose = robotPose3d.plus(shooterPoseTransform);
            var targetPose = getHubPose();

            var launchResult = calculateLaunch(
                    shooterPose,
                    targetPose.getTranslation(),
                    new Translation2d(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond),
                    pitchAngle,
                    Constants.fieldConstants.FUEL_BALL_MASS,
                    Constants.fieldConstants.FUEL_BALL_DIAMETER
            );

            launchAngularVelocity = launchResult.launchAngularVelocity(WHEEL_DIAMETER);
        } else {
            launchAngularVelocity = RevolutionsPerSecond.of(pref.shootingSpeedRPM());
        }

        return launchAngularVelocity;
    }

}
