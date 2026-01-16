package frc;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInstance;
import org.junit.jupiter.api.TestMethodOrder;
import org.junit.jupiter.api.TestInstance.Lifecycle;
import org.junit.jupiter.api.Timeout;
import org.opentest4j.TestAbortedException;

import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.helpers.MockHardwareExtension;
import frc.robot.Robot;


@TestInstance(Lifecycle.PER_CLASS)
@TestMethodOrder(MethodOrderer.OrderAnnotation.class)

/**
 * <h2> Class to run the stability tests of the robot in each mode.</h2>
 * Will create and enable a robot in both teliop and auto to make sure the robot can survive each mode without crashing. 
 * In the event of a failure, gradle will freak out so no checking is necessary beyond the afterTest in build.gradle

 */
public class test {


	Thread robotThread;
	IntegerSubscriber heartBeatSubscriber;
	long heartBeat=0;

	/**<h2>Creates the robot thread</h2> */
	@BeforeAll
	public void before() {
		// TestWithScheduler.schedulerStart();
		// TestWithScheduler.schedulerClear();
		MockHardwareExtension.beforeAll();

		robotThread = new Thread(()->RobotBase.startRobot(Robot::new));
		robotThread.setDaemon(true);


		

	}

	/**<h2>Checks if the robot can successfully boot </h2>*/
	@Test
	@Timeout(120)
	@Order(1)
	public void bootTest(){

		robotThread.start();

		try {
			Thread.sleep((long)Seconds.of(10).in(Milliseconds));
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
	}

	/**<h2>Checks if the robot can survive 30 seconds of teliop <h2> */
	@Test
	@Timeout(120)
	@Order(2)
	public void enableCheck() {
		assumeTrue(robotThread.isAlive());


		MockHardwareExtension.setTeliop();
		MockHardwareExtension.enable();


		try{
			Thread.sleep((long)Seconds.of(30).in(Milliseconds));
		}
		catch (InterruptedException e) {
			System.out.println("Interrupt");
		}

	}

	/**<h2>Checks if the robot can survive 30 seconds of autonomous <h2> */

	@Test
	@Timeout(120)
	@Order(3)
	
	public void autoCheck() {
		assumeTrue(robotThread.isAlive());

		MockHardwareExtension.setAuto();
		MockHardwareExtension.enable();

		try {
            Thread.sleep((long)Seconds.of(30).in(Milliseconds));
        } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

	}



	/**<h2>Cleans up and stops the robot thread </h2> */
	@AfterAll
	public void after() {
		MockHardwareExtension.afterAll();
		robotThread.stop();
	}
}
