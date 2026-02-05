package frc.robot.Utils;

import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;

public class QueryControllerConfig {
    
    public int getDeadzone() {
        JSONObject jsonMap=null;
        try{
            jsonMap = (JSONObject) new JSONParser().parse(Constants.controllerIDs.controllerConfigFilePath);
        }

        catch (ParseException e){
        }

        if (jsonMap==null){
            throw new Error("Json map was not able to be initalized, this is likely because the file \"/pathplanner/navgridAStar.json\" was not send with the robot.");
        }

    }

}
