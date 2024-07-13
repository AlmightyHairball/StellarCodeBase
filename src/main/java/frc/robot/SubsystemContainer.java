// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.BaseConstants.DriveConstants;
import frc.robot.RobotAutonomous.commands.Crescendo.Autos;
import frc.robot.RobotChassis.Subsystems.SwerveChassisSubsystem;
import frc.robot.RobotMechanisms.Crescendo.MechanismSubsystem;
import frc.robot.RobotMechanisms.Crescendo.Commands.DriveWithRotaryAndVisionLegacy;
import frc.robot.RobotMechanisms.Crescendo.Commands.MechanismControllerCommand;
import frc.robot.RobotVision.VisionSubsystem;

public class SubsystemContainer {

  // Declare the robot chassis
  private SwerveChassisSubsystem chassis;

  // Declare Auto Selector
  private SendableChooser<Command> autoChooser;

  // Add in Path planner field positioning telemetry
  private Field2d field;

  // Create experimental vision subsystem
  private VisionSubsystem vision = new VisionSubsystem();

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

    // Display a speed control
    SmartDashboard.putNumber("TranslationSpeed", DriveConstants.kMaxSpeedMetersPerSecond);
    SmartDashboard.putNumber("RotationSpeed", DriveConstants.kMaxAngularSpeedFactor);

    // Switches between vision and rotary for the 2024 robot
    chassis.setDefaultCommand(new DriveWithRotaryAndVisionLegacy(false, true, true, chassis));
    mechSystem.setDefaultCommand(new MechanismControllerCommand(mechSystem));

    // Set vision periodic to run periodicly
    vision.setDefaultCommand(new RunCommand(() -> {vision.periodic();}, vision));


    // Pathplanner logging
    field = new Field2d();
    SmartDashboard.putData("Field", field);
    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {field.setRobotPose(pose);});
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {field.getObject("target pose").setPose(pose);});
    PathPlannerLogging.setLogActivePathCallback((poses) -> {field.getObject("path").setPoses(poses);});

  }

  public SwerveChassisSubsystem getChassis() {
    return chassis;
  }

  public Field2d getFieldObject() {
    return field;
  }

  public Command getAutonomousCommand() {
    // Call the pathplanner auto lib
    return autoChooser.getSelected();
  }
}
