package frc;

import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

import edu.wpi.first.wpilibj.RobotBase;
import frc.helpers.MockHardwareExtension;
import frc.helpers.SchedulerPumpHelper;
import frc.helpers.TestWithScheduler;
import frc.robot.Robot;

public class test {
    
	@BeforeAll
	public static void before() {
		TestWithScheduler.schedulerStart();
		TestWithScheduler.schedulerClear();
		MockHardwareExtension.beforeAll();

	}

	// This test makes sure that the example command calls the .subsystemMethod of
	// example subsystem
	// Source - https://stackoverflow.com/q
// Posted by cmeiklejohn, modified by community. See post 'Timeline' for change history
// Retrieved 2026-01-08, License - CC BY-SA 4.0

	@Test
	@Timeout(120)

	public void enableCheck() throws InterruptedException {
		// Reset the subsystem to make sure all mock values are reset


		// Tell the button to run example command when pressed

		// Push the button and run the scheduler once
		    RobotBase.startRobot(Robot::new);
            MockHardwareExtension.enable();
            SchedulerPumpHelper.runForDuration(null, Seconds.of(30), 0);

            Thread.sleep(30000);
            MockHardwareExtension.disable();

	}

	// This test makes sure that periodic is called properly (odd case as this
	// should already work, but you may want to test methods inside of periodic)
	@Test
	public void ExampleSubsystemCallsPeriodic() {
		// Reset the subsystem to make sure all mock values are reset
        assertTrue(false);
	}

	// This is called after tests, and makes sure that nothing is left open and
	// everything is ready for the next test class
	@AfterAll
	public static void after() {
		TestWithScheduler.schedulerDestroy();
		MockHardwareExtension.afterAll();
	}
}
