package frc.robot.subsystems;

import java.io.FileReader;
import java.io.IOException;

import java.util.LinkedList;
import java.util.List;
import java.util.function.BooleanSupplier;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;
import com.pathplanner.lib.pathfinding.LocalADStar;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.FieldPosits;
import frc.robot.SystemManager;
import frc.robot.FieldPosits.reefLevel;
import frc.robot.FieldPosits.reefPole;
import frc.robot.FieldPosits.reefLevel.algaeRemovalEnum;
import frc.robot.Utils.scoringPosit;
import frc.robot.Utils.utilFunctions;
import frc.robot.commands.auto.IntakePieceCommand;
import frc.robot.commands.auto.ScorePiece;

import frc.robot.commands.auto.spin;

import frc.robot.commands.auto.removeAlgae;


public class autoManager{

    public static boolean hasControl=false;
    public static BooleanSupplier hasControlSupplier=null;
    public static Command currentRoutine=null;
    public static LocalADStar pathBuilder = new LocalADStar();
    public static Node[][] map;
    //public static boolean[][] legalityMap;
    public static JSONObject jsonMap ;
    protected static double tileSize;
    protected static double width;
    protected static double length;
    //static int resetCount=0;
    public static int cycleCount=0;
    public static int score=3;
    protected static StructPublisher<Pose2d> bestPosePublisher = NetworkTableInstance.getDefault().getStructTopic("bestPose", Pose2d.struct).publish();


    /**initializes the auto manager. this must be called before the auto manager is used */
    public static void autoManagerInit() {
        try{
            jsonMap = (JSONObject) new JSONParser().parse(new FileReader(Filesystem.getDeployDirectory()+ "/pathplanner/navgridAStar.json"));
        }
        catch (IOException e){
        }
        catch (ParseException e){
        }

        //lenArray = (JSONArray) jsonMap.get("field_size");
        width= (double)jsonMap.get("width");
        length=(double)jsonMap.get("length");
        tileSize=(double)jsonMap.get("nodeSizeMeters"); 

        // legalityMap = new boolean[(int)Math.ceil(width/tileSize)][(int)Math.ceil(length/tileSize)];
        map = new Node[(int)Math.ceil(width/tileSize)][(int)Math.ceil(length/(tileSize
        ))];
        JSONArray grid = (JSONArray)jsonMap.get("grid");
        for (int i=0; i<map.length; i++){
            JSONArray row = (JSONArray)grid.get(i);
            for (int j=0; j<map[i].length; j++){
                map[i][j] = map[i][j]=new Node(i/tileSize, j/tileSize, map, !(boolean)row.get(j));
                
                  
            }
        }

        map[0][0].popFriends();
        
    }


    /**handles the periodic tasks of the auto manager. should be called every cycle */
    public static void periodic(){
        SmartDashboard.putNumber("Cycle count", cycleCount);
        SmartDashboard.putNumber("Score", score);
        SmartDashboard.putNumber("updateCount", Node.updateCount);

        if (currentRoutine!=null){
            if (!currentRoutine.isScheduled()){
                currentRoutine=null;
            }
        }


        if (hasControlSupplier!=null){
            if (hasControlSupplier.getAsBoolean()!=hasControl){
                if (hasControlSupplier.getAsBoolean()==false){
                    takeControl();
                }
                else{
                    giveControl();
                }
            }
        }


        if (hasControl){
            if (currentRoutine==null){
                currentRoutine=getAutoAction();
                currentRoutine.schedule();
            }
        }

        if (currentRoutine==null){
            SmartDashboard.putString("autoRoutine", "null");
            
        }
        else{
            SmartDashboard.putString("autoRoutine", currentRoutine.getName());
        }
    }


    /**
     * swaps wether or not the auto manager has control
     * @param isGift wether or not the auto manager should gain control
     */
    public static void swapControl(boolean isGift){
        if (isGift){
            giveControl();
        }
        else{
            takeControl();
        }
    }

    /**gives the auto manager control */
    public static void giveControl(){
        hasControl=true;
    }

    /**takes control away from the auto manager */
    public static void takeControl(){
        hasControl=false;
        if (currentRoutine!=null){
            currentRoutine.cancel();
        }
    }

    /**
     * sets a supplier that the auto manager will use to determine if it has control. Note if this function is used the giveControl and take control methods may not work as intended.
     * @param supplier the supplier to determine wether the auto manager has control
     */
    public static void setControlBooleanSupplier(BooleanSupplier supplier){
        hasControlSupplier=supplier;    
    }


    /**
     * resets the internal A* map and recalculates paths using the starting pose
     * @param startingPose the pose to start the pathplanner on
     */
    protected static void resetMap(Pose2d startingPose){
        resetMap();
        getMapPoint(startingPose).start();
    }

    /**
     * resets the map but does not start a path.
     */
    protected static void resetMap(){

        for (int i=0; i<map.length; i++){
            for (int j=0; j<map[i].length; j++){
                map[i][j].reset();
            }          
        }
    }


    /**
     * fetches the closest node to the pose.
     * @param pose pose to check
     * @return the closest node to the pose
     */
    protected static Node getMapPoint(Pose2d pose){
        Pair<Integer, Integer> posit = poseToMap(pose);
        if (posit==null){
            return null;
        }
        return map[posit.getFirst()][posit.getSecond()];
    }

 
    /**
     * turns a pose into a int pair that can be used to index the internal map
     * @param pose the pose to map
     * @return a pair of ints representing the first and second index
     */
    public static Pair<Integer, Integer> poseToMap(Pose2d pose){
        if (pose.getY()>width || pose.getX()>length){
            return null;
        }
        return new Pair<Integer, Integer>((int)Math.round(pose.getY()/tileSize), (int)Math.round(pose.getX()/tileSize));
    }


    /** @return the best auto action to take at the frame called in the form of a command*/
    public static Command getAutoAction(){
        if (SystemManager.intake.hasPiece()){

            scoringPosit scorePose = getBestScorePosit();
            if (scorePose==null){
                return new spin();
            }

           
            if (SystemManager.reefIndexer.blockedByAlgae((int)scorePose.pole.getRowAsIndex(), scorePose.level.getAsInt())){
                
                return new removeAlgae(algaeRemovalEnum.makeFromNumbers((int)(scorePose.pole.getRowAsIndex())/2, (int)SystemManager.reefIndexer.getAlgaeLevel(scorePose.pole.getRowAsIndex()/2)-1));
                //return new removeAlgae(algaeRemoval.makeFromNumbers((int)(scorePose.pole.getRowAsIndex())/2+1, (int)SystemManager.reefIndexer.getAlgaeLevel((int)(scorePose.pole.getRowAsIndex()/2))-1));
            }
            return new ScorePiece(scorePose);

        }
        else{
            Pose2d intakePosit = getBestIntakePosit();
            if (intakePosit==null){
                return new spin();
            }
            return new IntakePieceCommand(intakePosit);
        }

    }

    /**
     * 
     * @return The best place to score in terms of points/time
     */
    public static scoringPosit getBestScorePosit(){
        resetMap(SystemManager.getSwervePose());
        
        scoringPosit winningPole=null;
        double winningScore=0;
        for(reefPole pole:FieldPosits.scoringPosits.scoringPoles){

            int highLevel = getHighestLevelWithGuiIntegrated(pole.getRowAsIndex());
            if (highLevel==-1){
                continue;
            }
            scoringPosit currentPosit = new scoringPosit(reefLevel.CreateFromLevel(highLevel), pole);
            double score =  currentPosit.getPointValForItem()/(getMapPoint(currentPosit.getScorePose()).score*2+Constants.AutonConstants.bonusScore);

        
            if (SystemManager.reefIndexer.blockedByAlgae(pole.getRowAsIndex(), SystemManager.reefIndexer.getHighestLevelForRow(pole.getRowAsIndex())-1)){
                score = score*1.1;
            }
            if (score>winningScore){
                winningPole=currentPosit;
                winningScore=currentPosit.getPointValForItem()/(getMapPoint(pole.getScorePosit()).score*2+Constants.AutonConstants.bonusScore);
            }
        }
        if (winningPole!=null){
            bestPosePublisher.set(winningPole.getScorePose());
        }

        return winningPole;

    }

    /**
     * @return the best position to intake from in terms of time.
     */
    public static Pose2d getBestIntakePosit(){

        resetMap(SystemManager.getSwervePose());
        Pose2d bestPose = null;
        double bestScore=100000;
        int i = 0;
        for (Pose2d point: FieldPosits.IntakePoints.coralSpawnPoints){
            if (!SystemManager.coralArray.getIntakeStationAvail(i)){
                continue;
            }
            if (getMapPoint(point).score<bestScore){
                bestPose=point;
                bestScore=getMapPoint(point).score;
            }
            i++;
        }
        if (bestPose!=null){
            bestPosePublisher.set(bestPose);
        }
        return bestPose;


    }

    public static int getHighestLevelWithGuiIntegrated(int level){
        boolean[] reefArr=SystemManager.reefIndexer.getFullReefState()[level];
        boolean[] guiArr = utilFunctions.flipBoolArray(SystemManager.coralArray.getGUIArray())[level];
        for (int i=3; i>=0;i--){
            if (!reefArr[i]&&guiArr[i]){
                return i;
            }
        }
        return -1;

    }
    
    /**class to represent a node in an aStar grid */
    public static class Node{
        public static double defaultValue=10000;
        public static int friendCount=0;
        public static int updateCount=0;
        public static List<Node> que = new LinkedList<Node>();

        double x, y;
        Node[][] host;
        boolean isLegal;
        double score=defaultValue;
        Node[] Friends = new Node[8];
        boolean friendsPopped=false;


        /**
         * creates a node
         * @param x the x position of the node in terms of list index
         * @param y the y position of the node in terms of list index
         * @param host The grid containing this node
         */
        public Node(double x, double y, Node[][] host){
            this(x,y,host,true);
        }

        /**
         * creates a node
         * @param x the x position of the node in terms of list index
         * @param y the y position of the node in terms of list index
         * @param host The grid containing this node
         * @param isLegal Wether or not the node is a legal place for a robot to be
         */
        public Node(double x, double y, Node[][] host, boolean isLegal){
            this.x=x;
            this.y=y;
            this.host=host;
            this.isLegal=isLegal;

        }

        /**populates this node with its friends. this function is recursive and so can generate the entire grid out of just one call */
        public void popFriends(){
            friendCount++;
            SmartDashboard.putNumber("friendCount", friendCount);
            
            int count=0;
            for (int i=-1; i<2; i++){
                for (int j=-1; j<2; j++){
                    if (i==0 && j==0){
                        continue;
                    }
                    int a=(int)(x*tileSize)+i;
                    int b = (int)(y*tileSize)+j;
                    if (a>=0 && a<map.length){
                        if (b>=0&& b<map[1].length){
                            Friends[count]=map[a][b];  
                        }

                        else{
                            Friends[count]=null;
                        }
                    }

                    else{
                        Friends[count]=null;
                    }

                    count++;
                }
               
            }
            
            
            friendsPopped=true;
            for (Node friend: Friends){
                if (friend!=null){
                    if (!friend.friendsPopped){
                        friend.popFriends();
                    }
                }
            }
        }

        /**updates this node based off its friend list */
        public void update(){
            updateCount++;
            
            
            for (Node friend: Friends){
                if (friend!=null&&friend.isLegal){
                    
                    if (friend.score>score+getLength(this, friend)){
                        friend.score=this.score+getLength(this, friend);
                        if (!que.contains(friend)){
                            que.add(friend);
                        }
                    }
                    
                }   
            }
        }

        /**
         * A handy function to make the calculation of the distance between two points easer
         * @param nodeA the first node
         * @param NodeB the second node
         * @return The distance between the two nodes
         */
        public static double getLength(Node nodeA, Node NodeB){
            return utilFunctions.pythagorean(nodeA.x, NodeB.x, nodeA.y, NodeB.y);
        }

        /**starts a pathplanning algorithm using this node as the starting point */
        public void start(){
            score=0;
            updateCount=0;
            que.clear();
            que.add(this);
            manage();
        }

        //resets this node to the default value
        public void reset(){
            score=defaultValue;
        }


        /**the internal function that handles pathplanning */
        public static void manage(){
            while(que.size()!=0){
                que.remove(0).update();;
            }
        }
    }

}
