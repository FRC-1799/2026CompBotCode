package frc.robot.subsystems.Intake;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.IntakeConstants;

public class realIntake extends Intake{

  private final SparkFlex intakeMotor = new SparkFlex(IntakeConstants.intakeCanID, MotorType.kBrushless);

  @Override
  public int getPieceCount() {
    return 0;
  }

  @Override
  protected void intake() {
    intakeMotor.set(IntakeConstants.intakeSpeed);
  }

  @Override
  protected void shutDown() {
    intakeMotor.set(0);
  }
 

  
}
