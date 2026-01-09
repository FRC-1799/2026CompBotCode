package frc;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInstance;
import org.junit.jupiter.api.TestMethodOrder;
import org.junit.jupiter.api.TestInstance.Lifecycle;
import org.junit.jupiter.api.Timeout;

import edu.wpi.first.wpilibj.RobotBase;
import frc.helpers.MockHardwareExtension;
import frc.helpers.TestWithScheduler;
import frc.robot.Robot;


@TestInstance(Lifecycle.PER_CLASS)
@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
public class test {
    
	Thread robotThread;

	


	@BeforeAll
	public void before() {
		TestWithScheduler.schedulerStart();
		TestWithScheduler.schedulerClear();
		MockHardwareExtension.beforeAll();

		robotThread = new Thread(()->RobotBase.startRobot(Robot::new));
		robotThread.setDaemon(true);

	}

	@Test
	@Timeout(120)
	@Order(1)
	public void bootTest() throws InterruptedException{

		robotThread.start();


		Thread.sleep((long)Seconds.of(10).in(Milliseconds));
		System.out.println(robotThread.isAlive());
		assertTrue(robotThread.isAlive());
		
	}

	@Test
	@Timeout(120)
	@Order(2)
	public void enableCheck() throws InterruptedException {

			
		    assumeTrue(robotThread.isAlive());

            MockHardwareExtension.setTeliop();
			MockHardwareExtension.enable();

            Thread.sleep((long)Seconds.of(30).in(Milliseconds));
            MockHardwareExtension.disable();
			assertTrue(robotThread.isAlive());

	}

	// This test makes sure that periodic is called properly (odd case as this
	// should already work, but you may want to test methods inside of periodic)
	@Test
	@Timeout(120)
	@Order(3)
	
	public void autoCheck() throws InterruptedException {
		// Reset the subsystem to make sure all mock values are reset
		System.out.println(robotThread.isAlive());
		assumeTrue(robotThread.isAlive());

		MockHardwareExtension.setAuto();
		MockHardwareExtension.enable();

		Thread.sleep((long)Seconds.of(30).in(Milliseconds));
		MockHardwareExtension.disable();
		assertTrue(robotThread.isAlive());
	}



	// This is called after tests, and makes sure that nothing is left open and
	// everything is ready for the next test class
	@AfterAll
	public static void after() {
		TestWithScheduler.schedulerDestroy();
		MockHardwareExtension.afterAll();
	}
}
