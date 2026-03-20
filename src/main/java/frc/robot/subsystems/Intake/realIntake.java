package frc.robot.subsystems.Intake;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.IntakeConstants;
import frc.robot.RobotPreferences;

public class realIntake extends Intake{

  protected RobotPreferences pref = RobotPreferences.getInstance();
  protected final SparkFlex intakeMotor = new SparkFlex(IntakeConstants.intakeCanID, MotorType.kBrushless);
 
  protected int indexerCount = 0;
  protected boolean indexerRun=true;


  @Override
  public int getPieceCount() {
    return 0;
  }

  @Override
  protected void intake() {
    intakeMotor.set(pref.intakeIngestSpeed());
  }

  @Override
  protected void shutDown() {
    intakeMotor.set(0);
  }

  @Override
  protected void backRun() {
    intakeMotor.set(pref.intakeBackwardSpeed());
  }

  @Override
  protected void indexing(){
     indexerCount--;
     if (indexerCount==0){
        indexerCount=10;
        indexerRun=!indexerRun;
     }

     intakeMotor.set(indexerRun? IntakeConstants.intakeIndexerSpeed: 0);
  }
 

  
}
