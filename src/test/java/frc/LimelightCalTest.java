package frc;

import org.junit.jupiter.api.Test;

import frc.robot.subsystems.vision.Calibration.LimelightCal;

import static org.junit.jupiter.api.Assertions.*;

class LimelightCalTest {

    @Test
    void createLimeLightCal() {
        var ids = new String[] {"id01", "id02"};

        LimelightCal cal = new LimelightCal(ids, 3, 10, 3, 0.25, 0.10);

        var thrown = assertThrows(
                IllegalArgumentException.class,
                () -> new LimelightCal(ids, 4, 10, 3, 0.25, 0.10),
                "did not detect bad count"
        );
    }
}