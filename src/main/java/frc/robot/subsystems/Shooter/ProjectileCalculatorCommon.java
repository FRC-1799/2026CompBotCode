package frc.robot.subsystems.Shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

import static edu.wpi.first.units.Units.*;

public class ProjectileCalculatorCommon {

    /**
     * Calculates the rotational velocity (RPM) from wheel surface linear velocity and wheel diameter.
     *
     * @param surfaceVelocity The linear velocity of the wheel surface.
     * @param wheelDiameter   The physical diameter of the wheel.
     * @return The angular velocity of the wheel wrapped in a WPILib type.
     */
    public static AngularVelocity calculateAngularVelocity(LinearVelocity surfaceVelocity, Distance wheelDiameter) {
        if (wheelDiameter.in(Meters) <= 0) {
            throw new IllegalArgumentException("Wheel diameter must be greater than zero.");
        }

        // 1. Convert inputs to standardized base SI units (Meters/Second and Meters)
        double velocityMetersPerSecond = surfaceVelocity.in(MetersPerSecond);
        double diameterMeters = wheelDiameter.in(Meters);

        // 2. Compute the wheel circumference
        double circumferenceMeters = Math.PI * diameterMeters;

        // 3. Calculate revolutions per second: v / circumference
        double revolutionsPerSecond = velocityMetersPerSecond / circumferenceMeters;

        // 4. Convert revolutions per second to revolutions per minute (RPM)
        double rpmValue = revolutionsPerSecond * 60.0;

        // Return wrapped explicitly as RPM
        return RPM.of(rpmValue);
    }

    /**
     * Calculates the surface linear velocity from a wheel's rotational velocity and diameter.
     *
     * @param angularVelocity The rotational speed of the wheel (e.g., RPM).
     * @param wheelDiameter   The physical diameter of the wheel.
     * @return The linear velocity of the wheel surface wrapped in a WPILib type.
     */
    public static LinearVelocity calculateLinearVelocity(AngularVelocity angularVelocity, Distance wheelDiameter) {
        if (wheelDiameter.in(Meters) <= 0) {
            throw new IllegalArgumentException("Wheel diameter must be greater than zero.");
        }

        // 1. Convert inputs to standardized base units (RPM and Meters)
        double rpmValue = angularVelocity.in(RPM);
        double diameterMeters = wheelDiameter.in(Meters);

        // 2. Compute the wheel circumference
        double circumferenceMeters = Math.PI * diameterMeters;

        // 3. Convert RPM to Revolutions Per Second
        double revolutionsPerSecond = rpmValue / 60.0;

        // 4. Calculate linear velocity: rps * circumference
        double velocityMetersPerSecond = revolutionsPerSecond * circumferenceMeters;

        // Return wrapped explicitly as MetersPerSecond
        return MetersPerSecond.of(velocityMetersPerSecond);
    }
}
