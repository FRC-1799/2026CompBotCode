package frc.robot.Utils;

import java.io.FileReader;
import java.io.IOException;
import java.lang.reflect.Array;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import java.util.stream.Collectors;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;
import org.opencv.core.Point;
import org.apache.commons.math3.fitting.PolynomialCurveFitter;
import org.apache.commons.math3.fitting.WeightedObservedPoints;

import frc.robot.Constants;

public class QueryControllerConfig {
    protected double deadzone;
    protected JSONObject jsonMap;
    protected double[] xControllerInputs;
    protected double[] yControllerInputs;
    protected double[] coeff;

    public QueryControllerConfig() {
        jsonMap=null;
        deadzone = Constants.controllerIDs.defaultDeadzone;

        try {
            jsonMap = (JSONObject) new JSONParser().parse(Constants.controllerIDs.controllerConfigFilePath);
        }

        catch (ParseException e) {
            throw new Error("Json map was not able to be initalized. This is probably because the file path does not exist");
        }

        deadzone = (double)jsonMap.get("deadzone");

        xControllerInputs = (double[])jsonMap.get("Points Array X");
        yControllerInputs = (double[])jsonMap.get("Points Array Y");

        WeightedObservedPoints points = new WeightedObservedPoints();

        for (int i = 0; i < Array.getLength(xControllerInputs) && i < Array.getLength(yControllerInputs); i++) {
            points.add(xControllerInputs[i], yControllerInputs[i]);
        }

        int degree = Constants.controllerIDs.degreeOfCurveFit;
        PolynomialCurveFitter fitter = PolynomialCurveFitter.create(degree);

        coeff = fitter.fit(points.toList());
        





    }
    
    public double getDeadzone() {
        return deadzone;

    }

    public double getNewOutput(double controllerValue) {
        return coeff[0] + coeff[1] * controllerValue + coeff[2] * controllerValue * controllerValue + coeff[2] * controllerValue * controllerValue * controllerValue;

    }

}
