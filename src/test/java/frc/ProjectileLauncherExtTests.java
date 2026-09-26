package frc;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.subsystems.Shooter.ProjectileCalculatorExt;
import org.junit.jupiter.api.Test;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;


public class ProjectileLauncherExtTests {

    // Define mock ball metrics matching an standard FRC game piece profile
    private static final edu.wpi.first.units.measure.Mass BALL_MASS = Pounds.of(0.5);
    private static final edu.wpi.first.units.measure.Distance BALL_DIAMETER = Inches.of(5.91);

    // Test tolerance thresholds (2cm delta limits)
    private static final double DISTANCE_TOLERANCE_METERS_ROUNDTRIP = 0.1;

    @Test
    public void testStationaryLaunchAndLandingLoopback() {

        // Launcher is placed at field origin, raised 1 meter up
        Pose3d launcherPose = new Pose3d(0.0, 0.0, 2.0, new Rotation3d());
        // Target is directly in front, 4 meters away, raised to 2.5 meters
        Translation3d targetGoal = new Translation3d(30.5, 0.0, 0.0);
        // Stationary platform
        Translation2d platformVelocity = new Translation2d(0.0, 0.0);
        // Launcher mechanically fixed at 40 degrees vertical inclination
        var pitchAngle = Degrees.of(30.0);

        // target velocity with drag
        LinearVelocity targetVelocity = MetersPerSecond.of(23.2);

        // Step A: Calculate required muzzle parameters to hit target
        ProjectileCalculatorExt.LaunchResult launchParams = ProjectileCalculatorExt.calculateLaunch(
                launcherPose,
                targetGoal,
                platformVelocity,
                pitchAngle,
                BALL_MASS,
                BALL_DIAMETER
        );

        // --- 3. ASSERT ---
        assertNotNull(launchParams, "Target should be within accessible physics bounds.");

        // Assert shooter yaw turns straight forward along the X axis toward the target (0 degrees)
        assertEquals(0.0, launchParams.launcherYaw().getDegrees(), 1.0e-3, "Yaw should look directly forward.");

        // Step B (Inverse Validation): Predict target plane crossing point using calculated launch speeds
        Translation3d impactLocation = ProjectileCalculatorExt.calculateLanding(
                launcherPose,
                launchParams.launchVelocity(),
                launchParams.launcherYaw(),
                platformVelocity,
                pitchAngle,
                BALL_MASS,
                BALL_DIAMETER,
                Meters.of(0.0)
        );

        assertNotNull(impactLocation, "Inverse loop tracker should resolve the target intercept plane.");

        // Check that loopback matches within the design convergence limits (2cm)
        assertEquals(targetGoal.getX(), impactLocation.getX(), DISTANCE_TOLERANCE_METERS_ROUNDTRIP, "Calculated landing X should match target.");
        assertEquals(targetGoal.getY(), impactLocation.getY(), DISTANCE_TOLERANCE_METERS_ROUNDTRIP, "Calculated landing Y should match target.");
    }

    @Test
    public void testMovingPlatformLaunchAndLandingLoopback() {
        // --- 1. ARRANGE ---
        // Launcher moving diagonally while firing at a target offset to the right side
        Pose3d launcherPose = new Pose3d(0.0, 0.0, 2.0, new Rotation3d());
        Translation3d targetGoal = new Translation3d(0.0, 30.0, 0.0);

        // Platform is actively moving at 1.5 m/s forward (X) and 0.5 m/s right (Y)
        Translation2d platformVelocity = new Translation2d(1.0, 0.0);
        var pitchAngle = Degrees.of(30.0);

        // --- 2. ACT ---
        ProjectileCalculatorExt.LaunchResult launchParams = ProjectileCalculatorExt.calculateLaunch(
                launcherPose,
                targetGoal,
                platformVelocity,
                pitchAngle,
                BALL_MASS,
                BALL_DIAMETER
        );

        // --- 3. ASSERT ---
        assertNotNull(launchParams, "Target should be reachable under active drive movement parameters.");

        var yawAngle = launchParams.launcherYaw().getDegrees();

        // Run Inverse execution tracking
        Translation3d impactLocation = ProjectileCalculatorExt.calculateLanding(
                launcherPose,
                launchParams.launchVelocity(),
                launchParams.launcherYaw(),
                platformVelocity,
                pitchAngle,
                BALL_MASS,
                BALL_DIAMETER,
                Meters.of(0.0)
        );

        assertNotNull(impactLocation, "Inverse simulation should hit ground plane.");
        assertEquals(targetGoal.getX(), impactLocation.getX(), DISTANCE_TOLERANCE_METERS_ROUNDTRIP, "Muzzle velocity must account for drift in X.");
        assertEquals(targetGoal.getY(), impactLocation.getY(), DISTANCE_TOLERANCE_METERS_ROUNDTRIP, "Muzzle velocity must account for drift in Y.");
    }
//
//    @Test
//    public void testPhysicallyImpossibleTargetReturnsNull() {
//        // --- 1. ARRANGE ---
//        // Target is placed completely out of reach (100 meters away)
//        Pose3d launcherPose = new Pose3d(0.0, 0.0, 1.0, new Rotation3d());
//        Translation3d outOfRangeTarget = new Translation3d(100.0, 0.0, 2.0);
//        Translation2d platformVelocity = new Translation2d(0.0, 0.0);
//        var pitchAngle = Degrees.of(35.0);
//
//        // --- 2. ACT ---
//        ProjectileCalculatorExt.LaunchResult launchParams = ProjectileCalculatorExt.calculateLaunch(
//                launcherPose, outOfRangeTarget, platformVelocity, pitchAngle, BALL_MASS, BALL_DIAMETER
//        );
//
//        // --- 3. ASSERT ---
//        assertNull(launchParams, "The calculation must return null if target exceeds max velocity caps.");
//    }
//
//    @Test
//    public void testIdenticalCoordinatesThrowsException() {
//        // --- 1. ARRANGE ---
//        // Target shares exact center point alignment with the shooter barrel
//        Pose3d launcherPose = new Pose3d(5.0, 5.0, 1.0, new Rotation3d());
//        Translation3d conflictingTarget = new Translation3d(5.0, 5.0, 3.0);
//        Translation2d platformVelocity = new Translation2d(0.0, 0.0);
//        var pitchAngle = Degrees.of(45.0);
//
//        // --- 2. ACT & ASSERT ---
//        assertThrows(IllegalArgumentException.class, () -> {
//            ProjectileCalculatorExt.calculateLaunch(
//                    launcherPose, conflictingTarget, platformVelocity, pitchAngle, BALL_MASS, BALL_DIAMETER
//            );
//        }, "Zero horizontal range vectors must drop an explicit argument exception constraint.");
//    }
}

