package frc;

import static org.junit.jupiter.api.Assertions.*;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import frc.robot.subsystems.Shooter.ProjectileCalculator;
import frc.robot.subsystems.Shooter.ProjectileCalculatorExt;
import org.junit.jupiter.api.Test;

import java.util.logging.Logger;

public class ProjectileLauncherTests {
    private static final Logger log = Logger.getLogger(ProjectileLauncherTests.class.getName());

    // Test tolerance thresholds (2cm delta limits)
    private static final double DISTANCE_TOLERANCE_METERS = 0.02;
    private static final double LINEAR_VELOCITY_TOLERANCE_METERS_SEC = 0.02;

    @Test
    public void calculateLaunchAndLandingTest() {

        Distance targetDistance = Meters.of(30.5);
        LinearVelocity targetVelocity = MetersPerSecond.of(17.6);

        var neededVelocity = ProjectileCalculator.calculateLaunch(
                Meters.of(2.0),
                Meters.of(0.0),
                Degrees.of(30),
                targetDistance
                );

        assertEquals(targetVelocity.in(MetersPerSecond), neededVelocity.in(MetersPerSecond), LINEAR_VELOCITY_TOLERANCE_METERS_SEC, "Calculated launch velocity should match target.");

        var result = ProjectileCalculator.calculateLanding(
                neededVelocity,
                Degrees.of(30),
                Meters.of(2.0),
                Meters.of(0.0)
        );

        assertEquals(result.landingDistance().in(Meters), targetDistance.in(Meters), DISTANCE_TOLERANCE_METERS, "Calculated landing distance should match target.");
    }

    @Test
    public void calculateLaunchErrorTest() {

        Distance targetDistance = Meters.of(1);

        var neededVelocity = ProjectileCalculator.calculateLaunch(
                Meters.of(0.0),
                Meters.of(2.0),
                Degrees.of(30),
                targetDistance
        );

        assertNull(neededVelocity, "Gave impossible launch angle.");
    }
}
