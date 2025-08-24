// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

    /**
     * Converts inches into meters
     * @param inches the number of inches
     * @return the number of meters
     */
    private static double inchesToMeters(double inches){
        return inches*0.0254;
    }


    public static final double LOOP_TIME  = 0.02; //s, 20ms + 110ms spark max velocity lag
    public static final int blinkinPort = 0;
    

    /**the constants for the drive train */
    public static final class driveConstants {
      public static final double driveFrictionVoltage = 0.25;
      public static final double steerFrictionVoltage = 0.25;
      public static final double totalWidth=inchesToMeters(27);
      public static final double totalHeight=inchesToMeters(27);
      public static final double chassisWidth=inchesToMeters(27);
      public static final double chassisHeight=inchesToMeters(27);
      public static final double steerInertia = 0.025;
      public static final double wheelRadiusInMeters = inchesToMeters(2);
      public static final double robotMass = 50;//(33) * 0.453592; // 32lbs * kg per pound
      public static final double maxSpeed  = 4.6;
      public static final Matter chassis    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(3)), robotMass);
      public static final double MOI = 6.884;
      public static final Pose2d startingPosit =  new Pose2d(7.182, 7.601, Rotation2d.fromDegrees(0));
      public static final double wheelLockTime = 10;
  }
   
  /**the constants for automation */
  public static final class AutonConstants{

    // public static final PIDConstants TRANSLATION_PID = new PIDConstants(10, 0, 0.5);
    public static final PIDConstants translationPID = new PIDConstants(10, 0, 0);
    public static final PIDConstants smallAutoPID = new PIDConstants(1.0, 0, 0.5);

    public static final PIDConstants anglePID       = new PIDConstants(5, 0, 0);


    public static final double collisionSpeed = 0;

    public static final double widthOfMapMeters=8;
    public static final double heightOfMapMeters=8.9;
    public static final double bonusScore=1;

    public static final double autoDriveScoreTolerance  = 0.5;
    public static final double autoDriveIntakeTolerance = 0.1;
    public static final double distanceWithinPathplannerDontWork = 0.5;

    public static final double humanPlayerBeingBad = 0.5;
    public static final Transform2d intakeCoralOffset = new Transform2d(-0.47, 0, new Rotation2d());
    public static final int autoDriveCorrectCount = 3;
  }

  /**constants for the controls */
  public static class OperatorConstants{
    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
    public static final int[] supportedPOV={0,90,180,270};
  }

  /**Constants used for sim */
  public static class sim{
    public static double targetTolerance =0.2;
    public static double l4CoralDropCheatX=0.08;
    public static double l4CoralDropCheatY=0.1;
  }

  /**constants to control wether or not a specific system should be simulated */
  public static class simConfigs{
    public static final boolean driveShouldBeSim=false||!RobotBase.isReal();
    public static final boolean intakeShouldBeSim=false||!RobotBase.isReal();
    public static final boolean aprilTagShouldBeSim=false||!RobotBase.isReal();
    public static final boolean wristShouldBeSim=false||!RobotBase.isReal();
    public static final boolean elevatorShouldBeSim=false||!RobotBase.isReal();

    public static final boolean reefIndexerShouldBeSim=false||!RobotBase.isReal();
    public static final boolean lidarShouldBeSim=true||!RobotBase.isReal();
    public static final boolean blinkinShouldBeSim=true||!RobotBase.isReal();
    public static final boolean algaeRemoverShouldBeSim=false||!RobotBase.isReal();
    public static final boolean robotCanBeSimOnReal=true;

    

  }

  /**ids of all used controllers */
  public static class controllerIDs{
    public static final int commandXboxController1ID=0;
    public static final int commandXboxController2ID=1;
  }

  /**constants for the intake */
  public static class intakeConstants{
    public static final double intakeSpeed=0.3;
    public static final double outtakeSpeed=1;
    public static final double intakeLength=0.245745;
    public static final int topIntake = 10;
    public static final int bottomIntake = 7;
    public static final double coralFromWristLen = 0.1715772;
    public static final double coralLength = 0.3;
    public static final double coralWidth = 0.11;


    public static final int frontBeamBrakePort=9;
    public static final int backBeamBrakePort=8;

   

  }

  public static final class algaeRemoverConstants{
    public static final double algaeTimerVal = 10;
    public static final int motorID=12;

    public static final double motorSpeed = 0.7;
  }


  /**constants for the elevator */
  public static class elevatorConstants{
    public static final double l4EncoderVal = 1.42;
    public static final double l3EncoderVal = 1.02381;
    public static final double l2EncoderVal = 0.6514528;

    public static final double l1EncoderVal = 0.37;
    public static final double encoderToMeters =69.1514; //1/141;
    public static final int resetSwitchID = 7;


    public static final double highAlgaePrep = 0.8;
    public static final double lowAlgaePrep = 0;
    // public static final double lowAlgaeAct = 0;
    // public static final double highAlgaeAct = 0;



    public static final double maxHeight = l4EncoderVal;
    public static final Angle angle = Degrees.of(110);
    public static final double intakePosit = 0;
    public static final double startingPosit = 0;
    public static final double tolerance = 0.05;
    public static final double speedForSim =0.02;
    public static final double compressedLen = 0.889;
    public static final PIDConstants elevatorPID = new PIDConstants(1.7, 0, 0);
    //public static final PIDConstants elevatorPID = new PIDConstants(0.5, 0, 0);

    //public static final double g=0.11;
    public static final double g=0.03;


    public static final boolean shouldUseCurrentEncoderReset=true;
    public static final double currentResetThreshold=15;


    public static final int mainMotorID=2;
    public static final int altMotorID=1;


    // in init function

    // set slot 0 gains
    public static final Slot0Configs slot0Configs = new Slot0Configs()
        .withKS(0.25) // Add 0.25 V output to overcome static friction
        .withKV(0.35) // A velocity target of 1 rps results in 0.12 V output
        .withKA(0.15) // An acceleration of 1 rps/s requires 0.01 V output
        .withKP(7.4) // A position error of 2.5 rotations results in 12 V output
        .withKI(0) // no output for integrated error
        .withKD(0.005) // A velocity error of 1 rps results in 0.1 V output
        .withKG(1.8)
    ;
    public static final double elevatorResetTolerance = 0.3;
    public static final double elevatorIntakeEndOffset = 0.4;
        public static final Translation3d fromRobotCenter = new Translation3d(-0.0584454, 0, 0.583565 );


    // set Motion Magic settings
    public static MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
        .withMotionMagicAcceleration(160)// Target acceleration of 160 rps/s (0.5 seconds)
        .withMotionMagicCruiseVelocity(80)// Target cruise velocity of 80 rps
        .withMotionMagicJerk(1600)// Target jerk of 1600 rps/s/s (0.1 seconds)
        .withMotionMagicExpo_kA(0.2)
    ;

    
  }

  /**constants for the wrist */
  // public static class wristConstants{
  //   public static final Rotation2d l4EncoderVal = Rotation2d.fromDegrees(210);
  //   public static final Rotation2d l3EncoderVal = Rotation2d.fromDegrees(0);
  //   public static final Rotation2d l2EncoderVal = Rotation2d.fromDegrees(0);
  //   public static final Rotation2d l1EncoderVal = Rotation2d.fromDegrees(0);
  //   public static final Rotation2d restingPosit = Rotation2d.fromDegrees(0);
  //   public static final Rotation2d intakePosit = Rotation2d.fromDegrees(0);
  //   public static final Rotation2d algaePosit = Rotation2d.fromDegrees(0);

  //   public static final Rotation2d minDegrees = new Rotation2d();
  //   public static final Rotation2d maxDegrees = new Rotation2d();
    
  //   public static final double tolerance = 3;
  //   public static final double speedForSim=5;
  //   public static final int CANCoderID=3;
  //   public static final int throughBoreID = Constants.intakeConstants.topIntake;
  //   public static final double CANCoderOffset=0;//5.4;
  //   public static final int motorID=8;
  //   public static final double fConstant=0.01;
  //   public static final PIDConstants wristPID = new PIDConstants(0.0015, 0, 0.);
  //   public static final double maxVel = 0;
  //   public static final double maxAccel = 0;
  
  // }

  //constants used for the simulated camera
  public static class cameraConstants{
    public static Transform3d frontAprilTagCameraTrans = new Transform3d();

  }

}
