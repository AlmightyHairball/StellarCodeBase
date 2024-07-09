// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotMechanisms.Crescendo.Subsystems;

import java.util.Dictionary;
import java.util.Hashtable;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMechanisms.Crescendo.MechanismConstants.MiscConstants;

public class LightingSubsystem extends SubsystemBase {

  // Define the blinken module and the presetlist
  private final Spark lightController = new Spark(MiscConstants.lightControllerPort);

  // Create a dictionary to store presets
  final Dictionary<String, Double> lightList = new Hashtable<>();

  // Create a reserve variable to keep mechanisms from fighting over the lights
  public boolean isReserved = false;

  public LightingSubsystem() {

  }

    /* This class is utilizing the singleton pattern
   * because there should be no other SubsystemContainer 
   * other than the one instance that this class hands out. */

  // Create singleton instance
  private static LightingSubsystem instance;

  // Create method for getting singleton instance.
  // If theres no instance, create one, then return the instance.
  public static LightingSubsystem getSingletonInstance() {
    if (instance == null) {
      instance = new LightingSubsystem();
      instance.init();
    }
    return instance;
  }

  // Used for initiating the singleton subsystem
  private void init() {

    this.setPrimary();

    // Add presets to the dictionary
    lightList.put("solid_yellow", 0.69);
    lightList.put("solid_blue", 0.87);
    lightList.put("solid-red", 0.61);
    lightList.put("solid-green", 0.77);
    lightList.put("solid-pink", 0.57);

  }


  public void setManualy(double PWMVal) {
      lightController.set(PWMVal);
  }

  public void storedPreset(String presetName) {
      lightController.set(lightList.get(presetName));
  }

  public void setPrimary() {
      lightController.set(0.39);
  }

  public void setSecondary() {
      lightController.set(0.37);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
