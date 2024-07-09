// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotAutonomous.commands.Crescendo.Autos;
import frc.robot.RobotChassis.Subsystems.SwerveChassisSubsystem;
import frc.robot.RobotMechanisms.Crescendo.MechanismSubsystem;
import frc.robot.RobotMechanisms.Crescendo.Commands.DriveWithRotaryAndVisionLegacy;
import frc.robot.RobotMechanisms.Crescendo.Commands.MechanismControllerCommand;

public class SubsystemContainer {

  // Declare the robot chassis
  private SwerveChassisSubsystem chassis;

  // Declare Auto Selector
  private SendableChooser<Command> autoChooser;

public SubsystemContainer() { 
  /* Initialization code is handled by calling initializeRobot in the Robot class */
}

  /* This class is utilizing the singleton pattern
   * because there should be no other SubsystemContainer 
   * other than the one instance that this class hands out. */

  // Create singleton instance
  private static SubsystemContainer instance;

  // Create method for getting singleton instance.
  // If theres no instance, create one, then return the instance.
  public static SubsystemContainer getSingletonInstance() {
    if (instance == null) {
      instance = new SubsystemContainer();
    }
    return instance;
  }


  public void initiateRobot() {
    /* ----------------------------------------------------------------------------------------
    *  The code below should be replaced with a command friendly method
    * of creating button binds, but for now, I'm too lazy. (PARTIALLY FIXED)
    * ---------------------------------------------------------------------------------------- */

    // Define the Chassis
    chassis = new SwerveChassisSubsystem();

    // Define 2024 robot Mechansisms
    MechanismSubsystem mechSystem = MechanismSubsystem.getSingletonInstance();

    // Commands should be registered before the auto chooser
    Autos.registerCommandsCrescendo(chassis, mechSystem, mechSystem.getVisionObject());

    // Display the Auto Selector
    autoChooser = AutoBuilder.buildAutoChooser("Default");
    SmartDashboard.putData("Select Auto", autoChooser);

    // Switches between vision and rotary for the 2024 robot
    chassis.setDefaultCommand(new DriveWithRotaryAndVisionLegacy(false, true, true, chassis));
    mechSystem.setDefaultCommand(new MechanismControllerCommand(mechSystem));

  }

  public SwerveChassisSubsystem getChassis() {
    return chassis;
  }

  public Command getAutonomousCommand() {
    // Call the pathplanner auto lib
    return autoChooser.getSelected();
  }
}
