package frc.robot.Utils;

import java.io.FileReader;
import java.io.IOException;
import java.util.LinkedList;
import java.util.List;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LocalAStar {
    public static Node[][] map;
    protected static double tileSize;
    protected double width;
    protected double length;

    public LocalAStar(){
        JSONObject jsonMap=null;
        try{
            jsonMap = (JSONObject) new JSONParser().parse(new FileReader(Filesystem.getDeployDirectory()+ "/pathplanner/navgridAStar.json"));
        }
        catch (IOException e){
        }
        catch (ParseException e){
        }

        if (jsonMap==null){
            throw new Error("Json map was not able to be initalized, this is likely because the file \"/pathplanner/navgridAStar.json\" was not send with the robot.");
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
    
    /**
     * fetches the closest node to the pose.
     * @param pose pose to check
     * @return the closest node to the pose
     */
    public Node getMapPoint(Pose2d pose){
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
    public Pair<Integer, Integer> poseToMap(Pose2d pose){
        if (pose.getY()>width || pose.getX()>length){
            return null;
        }
        return new Pair<Integer, Integer>((int)Math.round(pose.getY()/tileSize), (int)Math.round(pose.getX()/tileSize));
    }


    /**
     * resets the map but does not start a path.
     */
    public void resetMap(){

        for (int i=0; i<map.length; i++){
            for (int j=0; j<map[i].length; j++){
                map[i][j].reset();
            }          
        }
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
