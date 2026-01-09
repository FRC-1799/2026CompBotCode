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

import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.helpers.MockHardwareExtension;
import frc.helpers.TestWithScheduler;
import frc.robot.Robot;


@TestInstance(Lifecycle.PER_CLASS)
@TestMethodOrder(MethodOrderer.OrderAnnotation.class)

public class test {


	Thread robotThread;
	IntegerSubscriber heartBeatSubscriber;
	long heartBeat=0;


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
		heartBeatSubscriber = NetworkTableInstance.getDefault().getIntegerTopic("/SmartDashboard/heartbeat").subscribe(0);
		
		Thread errorThread = new Thread(()->{throw new Error();});
		errorThread.start();
		
		heartBeat=heartBeatSubscriber.get();
		Thread.sleep((long)Seconds.of(9).in(Milliseconds));
		assertTrue(heartBeatSubscriber.get()>heartBeat);


		heartBeat=heartBeatSubscriber.get();
		Thread.sleep((long)Seconds.of(1).in(Milliseconds));
		assertTrue(heartBeatSubscriber.get()>heartBeat);
		
	}

	@Test
	@Timeout(120)
	@Order(2)
	public void enableCheck() {
		//assumeTrue(robotThread.isAlive());


		MockHardwareExtension.setTeliop();
		MockHardwareExtension.enable();

		System.out.println("hiiiii");

		heartBeat=heartBeatSubscriber.get();
		System.out.println("hiiiii:3");
		try{
			Thread.sleep((long)Seconds.of(0.1).in(Milliseconds));
		}
		catch (InterruptedException e) {
			System.out.println("Interrupt");
		}
		System.out.println("hiiiii:3:3");
		// for (int i=0;i<10000;i++){System.out.println(heartBeat);}

	

		
		assertTrue(heartBeatSubscriber.get()>heartBeat);



		MockHardwareExtension.disable();
		heartBeat=heartBeatSubscriber.get();
		try {
			Thread.sleep((long)Seconds.of(1).in(Milliseconds));
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}


		assertTrue(heartBeatSubscriber.get()>heartBeat);

	}

	// This test makes sure that periodic is called properly (odd case as this
	// should already work, but you may want to test methods inside of periodic)
	@Test
	@Timeout(120)
	@Order(3)
	
	public void autoCheck() throws InterruptedException {
		// Reset the subsystem to make sure all mock values are reset
		assumeTrue(robotThread.isAlive());

		MockHardwareExtension.setAuto();
		MockHardwareExtension.enable();

		heartBeat=heartBeatSubscriber.get();
		Thread.sleep((long)Seconds.of(29).in(Milliseconds));
		assertTrue(heartBeatSubscriber.get()>heartBeat);
		
		MockHardwareExtension.disable();
		heartBeat=heartBeatSubscriber.get();
		Thread.sleep((long)Seconds.of(1).in(Milliseconds));
		assertTrue(heartBeatSubscriber.get()>heartBeat);
	}



	// This is called after tests, and makes sure that nothing is left open and
	// everything is ready for the next test class
	@AfterAll
	public void after() {
		MockHardwareExtension.afterAll();
		robotThread.stop();
	}
}
