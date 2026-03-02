// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;


/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what you are doing, do
 * not modify this file except to change the parameter class to the startRobot call.
 */
public final class Main
{

  private Main()
  {
  }

  /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>If you change your main robot class, change the parameter type.
   */
  public static void main(String... args)
  {
    try {
      var versionMsg = String.format("Version: %s Branch: %s %s\nBuilt on: %s\n", BuildConstants.VERSION, BuildConstants.GIT_BRANCH, BuildConstants.GIT_SHA.substring(0, 6), BuildConstants.BUILD_DATE);
      System.out.print(versionMsg);
    }
    catch(RuntimeException e) {
      System.out.println("Version: Unknown (error getting values)");
    }
    
    RobotBase.startRobot(Robot::new);
  }
}
