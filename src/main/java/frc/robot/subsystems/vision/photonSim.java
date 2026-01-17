package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SystemManager;

public class photonSim extends SubsystemBase implements aprilTagInterface{
    VisionSystemSim visionSim;

    /**
     * creates a new simulated vision system. 
     * While it is possible to have simulated vision running on the real robot it is not recommended since the system does nothing and takes up a lot of processing time
    */
    public photonSim(){
        visionSim = new VisionSystemSim("main");
        //AprilTagFieldLayout tagLayout = AprilTagFields

        visionSim.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded));
        // A 640 x 480 camera with a 100 degree diagonal FOV.
        SimCameraProperties  cameraProp = new SimCameraProperties();  
        cameraProp.setCalibration(1280, 800, Rotation2d.fromDegrees(70));
        // Approximate detection noise with average and standard deviation error in pixels.
        cameraProp.setCalibError(0.25, 0.08);
        // Set the camera image capture framerate (Note: this is limited by robot loop rate).
        cameraProp.setFPS(100);
        // The average and standard deviation in milliseconds of image data latency.
        cameraProp.setAvgLatencyMs(10);
        cameraProp.setLatencyStdDevMs(6);

        PhotonCamera camera = new PhotonCamera("cameraName");

        // The simulation of this camera. Its values used in real robot code will be updated.
        PhotonCameraSim cameraSim = new PhotonCameraSim(camera, cameraProp);


        

        // Add this camera to the vision system simulation with the given robot-to-camera transform.
        visionSim.addCamera(cameraSim, Constants.cameraConstants.frontAprilTagCameraTrans);



        //cameraSim.enableDrawWireframe(true);

    }

    @Override
    public Pose3d getFrontPose(){
        return visionSim.getRobotPose();
    }

    @Override
    public Pose3d getBackPose(){
        return visionSim.getRobotPose();
    }

    /**periodic call that updates the vision sim */
    @Override
    public void periodic(){
        visionSim.update(SystemManager.getRealPoseMaple());
    }

    @Override
    public Double getFrontTimestamp() { 
        return Timer.getFPGATimestamp();
    }

    @Override
    public Double getBackTimestamp() {
        return Timer.getFPGATimestamp();
    }
}
