package frc;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.subsystems.Shooter.ProjectileCalculatorCommon;
import org.junit.jupiter.api.Test;
import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.assertEquals;

public class ProjectileCalculatorCommonTests {

    // Test tolerance thresholds (2cm delta limits)
    private static final double ANGULAR_VELOCITY_TOLERANCE_RPM = 0.05;
    private static final double LINEAR_VELOCITY_TOLERANCE_METERS_PER_SEC = 0.05;

    @Test
    public void calculateRPMTest() {
        LinearVelocity surfaceVelocity = MetersPerSecond.of(13.3);
        Distance wheelDiameter = Meters.of(0.254);
        AngularVelocity expectedRPM = RPM.of(1000);

        AngularVelocity calculatedRPM = ProjectileCalculatorCommon.calculateAngularVelocity(surfaceVelocity, wheelDiameter);

        assertEquals(expectedRPM.in(RPM),calculatedRPM.in(RPM), ANGULAR_VELOCITY_TOLERANCE_RPM, "RPM isn't within tolerance");

        LinearVelocity calculatedVelocity = ProjectileCalculatorCommon.calculateLinearVelocity(calculatedRPM, wheelDiameter);

        assertEquals(surfaceVelocity.in(MetersPerSecond),calculatedVelocity.in(MetersPerSecond), ANGULAR_VELOCITY_TOLERANCE_RPM, "RPM isn't within tolerance");
    }
}
