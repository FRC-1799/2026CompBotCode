package frc.robot.subsystems.vision;


import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.Arena2025Reefscape;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeReefSimulation;


import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.FieldPosits;


public class simReefIndexer extends reefIndexerIO{
     

     int heartBeat=0;
     final boolean[][] algaeSource={{true,false},{false,true},{true,false},{false,true},{true,false},{false,true}};;
     boolean[][] algae = algaeSource;

    Arena2025Reefscape arena = (Arena2025Reefscape) SimulatedArena.getInstance();
        StructArrayPublisher<Pose3d> algaePublisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("reefAlgae", Pose3d.struct).publish();

    public simReefIndexer(){
    }

    @Override
    public void periodic(){
        Pose3d[] algaeRender = new Pose3d[12];
        for (int i=0; i<6;i++){
            for (int j=0;j<2;j++){
                //System.out.println(algae[i][j]);
                if (algae[i][j]){
                    algaeRender[i*2+j] = FieldPosits.algaeRenderPosits[i][j];
                }
                else{
                    algaeRender[i*2+j] = new Pose3d();//FieldPosits.algaeRenderPosits[i][j];
                }
                
            }
        }
        algaePublisher.set(algaeRender);
    }


    @Override
    public boolean[][] getFullReefState() {
        int[][]base = arena.blueReefSimulation.getBranches();
        boolean[][]returnable = new boolean[12][4];
        int i = 0;
        
        for(int[] branch : base){
            int j = 0;
            for(int item: branch){
                returnable[i][j] = item!=0;
                j++;
            }
            i++;
        }
        i=0;
        for (int[] pole: base){
            SmartDashboard.putNumberArray(String.valueOf(i), new double[]{pole[0],pole[1],pole[2],pole[3]});
            i++;
        }

        
        //reefBranch.set(publishArr);
        return returnable;
    }




    @Override
    public void resetSIMONLY(){
        arena.resetFieldForAuto();;
    }


    @Override 
    public boolean[][] getAlgaePosits(){
        
        return algae;
    }

    @Override
    public void freeAlgae(int row, int level){

        algae[row][level]=false;
    }




    
}
