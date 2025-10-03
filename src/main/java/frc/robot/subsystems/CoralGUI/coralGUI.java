package frc.robot.subsystems.CoralGUI;
import edu.wpi.first.networktables.BooleanArrayPublisher;
import edu.wpi.first.networktables.BooleanArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class coralGUI extends SubsystemBase {
    
    //private String[][] coralArray = new String[4][12];  // Array to hold the Coral data for 4 floors and 12 columns

    //NetworkTableInstance inst;
    //NetworkTable coralF1;
    //NetworkTable coralF2; 
    //NetworkTable coralF3;
    //NetworkTable coralF4;
    NetworkTableInstance table = NetworkTableInstance.getDefault();
    boolean[] defaultVals={true, true, true, true, true, true, true, true, true, true, true, true};
    boolean[] defaultIntakeVals = {true, true, true};
    BooleanArraySubscriber l1 = table.getBooleanArrayTopic("GUI/CoralL1").subscribe(defaultVals,  PubSubOption.keepDuplicates(true));
    BooleanArraySubscriber l2 = table.getBooleanArrayTopic("GUI/CoralL2").subscribe(defaultVals,  PubSubOption.keepDuplicates(true));
    BooleanArraySubscriber l3 = table.getBooleanArrayTopic("GUI/CoralL3").subscribe(defaultVals,  PubSubOption.keepDuplicates(true));
    BooleanArraySubscriber l4 = table.getBooleanArrayTopic("GUI/CoralL4").subscribe(defaultVals,  PubSubOption.keepDuplicates(true));
    BooleanArraySubscriber leftIntake = table.getBooleanArrayTopic("GUI/leftIntake").subscribe(defaultIntakeVals, PubSubOption.keepDuplicates(true));
    BooleanArraySubscriber rightIntake = table.getBooleanArrayTopic("GUI/rightIntake").subscribe(defaultIntakeVals, PubSubOption.keepDuplicates(true));

    BooleanArrayPublisher l1Test = table.getBooleanArrayTopic("GUItest/ReefL1").publish( PubSubOption.keepDuplicates(true));
    BooleanArrayPublisher l2Test = table.getBooleanArrayTopic("GUItest/ReefL2").publish(PubSubOption.keepDuplicates(true));
    BooleanArrayPublisher l3Test = table.getBooleanArrayTopic("GUItest/ReefL3").publish(  PubSubOption.keepDuplicates(true));
    BooleanArrayPublisher l4Test = table.getBooleanArrayTopic("GUItest/ReefL4").publish(  PubSubOption.keepDuplicates(true));

    public coralGUI() {
        // Initialize the network table instance
        // inst = NetworkTableInstance.getDefault();
        
        // // Retrieve the network tables for each floor
        // coralF1 = inst.getTable("GridData").getSubTable("CoralF1");
        // coralF2 = inst.getTable("GridData").getSubTable("CoralF2");
        // coralF3 = inst.getTable("GridData").getSubTable("CoralF3");
        // coralF4 = inst.getTable("GridData").getSubTable("CoralF4");

        // Update the coralArray with data from the network tables
        
    }

    @Override
    public void periodic(){
        
        boolean[][] coralArray = getGUIArray();
        l1Test.set(coralArray[0]);
        l2Test.set(coralArray[1]);
        l3Test.set(coralArray[2]);
        l4Test.set(coralArray[3]);
        
    }

    public boolean[][] getGUIArray(){
        boolean[][]coralArray = new boolean [4][12];
        // for (int i = 0; i < 12; i++) {
        //     coralArray[3][i] = l1.get(); // Floor 4 (top)
        //     coralArray[2][i] = coralF3.getEntry(Integer.toString(i)).getString("0"); // Floor 3
        //     coralArray[1][i] = coralF2.getEntry(Integer.toString(i)).getString("0"); // Floor 2
        //     coralArray[0][i] = coralF1.getEntry(Integer.toString(i)).getString("0"); // Floor 1 (bottom)
        // }
        coralArray[0]=l1.get();
        coralArray[1]=l2.get();
        coralArray[2]=l3.get();
        coralArray[3]=l4.get();
        return coralArray;
    }

    // Function to check if a specific floor and column contains a 1
    public boolean checkCoralValue(int level, int column) {
        if (level < 0 || level > 3 || column < 0 || column > 11) {
            throw new IllegalArgumentException("Floor must be between 1 and 4, Column must be between 1 and 12.");
        }
        // Convert floor to array index (floor 1 = index 0, floor 4 = index 3)
        
        // Check if the value is 1
        return getGUIArray()[level][column];
    }
    public boolean[] getIntakeList(){
        boolean[] leftIntakeVals = leftIntake.get();
        boolean[] rightIntakeVals =  rightIntake.get();
        return new boolean[]{leftIntakeVals[0], leftIntakeVals[1], leftIntakeVals[2], rightIntakeVals[0], rightIntakeVals[1], rightIntakeVals[2]};
    }

    public boolean getIntakeStationAvail(int intakeStation){
        return getIntakeList()[intakeStation];
    }
}
