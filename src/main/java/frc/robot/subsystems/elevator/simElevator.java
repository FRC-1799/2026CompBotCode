package frc.robot.subsystems.elevator;


import frc.robot.Constants;

public class simElevator  extends elevatorIO{


    public double position=0;
    protected double goal=0;
    

    @Override
    public double getEncoderVal() {
        return position;
    }

    @Override
    public double getHeight(){
        return position;
    }

    @Override
    public void periodic(){


        goal=setpoint;
       


        if (Math.abs(goal-position)<Constants.elevatorConstants.speedForSim){
            position=goal;
        }
        else if (goal>position){
            position+=Constants.elevatorConstants.speedForSim;
        }
        else{
            position-=Constants.elevatorConstants.speedForSim;
        }

        updateRender();
        
    }





    
    
}
