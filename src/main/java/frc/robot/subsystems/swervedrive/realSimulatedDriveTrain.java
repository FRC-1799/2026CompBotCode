package frc.robot.subsystems.swervedrive;


import frc.robot.SystemManager;
import swervelib.simulation.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import swervelib.simulation.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;


/**class to create a simulated drive train that mirrors the yagsl drivetrain. 
 * This class is necessary to have a simulated intake on a real robot since maplesim chassis actually handle all intake logic, not the simulated intake itself.
 * This class keeps all functionality necessary to run a simulated intake while removing all other drivetrain simulation.
 */
public class realSimulatedDriveTrain extends SwerveDriveSimulation{

    /**creates a real simulated drive train that can be passed to a simulated intake */
    public realSimulatedDriveTrain(){
        super(DriveTrainSimulationConfig.Default(), SystemManager.getSwervePose());
    }

    /**resets the simulated drivetrain's pose to the real drivetrain pose and stops all simulated logic from running */
    @Override
    public void simulationSubTick(){
        this.setSimulationWorldPose(SystemManager.getSwervePose());
    }
}
