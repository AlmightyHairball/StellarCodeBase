// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotMechanisms.Crescendo.Commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotControl.ControllerIO;
import frc.robot.RobotControl.ControllerIO.controllerType;
import frc.robot.RobotMechanisms.Crescendo.MechanismConstants.ShooterConstants;
import frc.robot.RobotMechanisms.Crescendo.Subsystems.LightingSubsystem;
import frc.robot.RobotMechanisms.Crescendo.MechanismSubsystem;
import frc.robot.RobotUtilities.MiscUtils;


// This is where button binds and primary logic should be for the 
// 2024 robot mechanisms (Epic).
public class MechanismControllerCommand extends Command {

  // Define our subsystems
  private final MechanismSubsystem mechSystem = MechanismSubsystem.getSingletonInstance();
  private final LightingSubsystem floodLights = LightingSubsystem.getSingletonInstance();

  // Variable to hold our controller object
  private final XboxController operatorController = ControllerIO.getSecondaryInstance(controllerType.XBOX).xboxController;

  public MechanismControllerCommand(MechanismSubsystem mechSystem) {
    // Use addRequirements() here to declare subsystem dependencies. (No requirements)
    addRequirements(mechSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Light logic
    if (!floodLights.isReserved) { // Make sure the intake isnt upset before updating
      if (mechSystem.getIntakePos() < 0.30) {
        if (mechSystem.getShooterSpeed() > 100) {

          if (mechSystem.getShooterSpeed() < 3600) {
            floodLights.storedPreset("solid-red");
            SmartDashboard.putString("Lights", "Red");
          } else {
            floodLights.storedPreset("solid-green");
            SmartDashboard.putString("Lights", "Green");
          }
        } else {
          floodLights.setPrimary();
          SmartDashboard.putString("Lights", "Idle");
        }
      }
    }



    // Mechanism Subsystem Controller Binds:
    // Create a quick refrence to the operator POV HAT.
    int operatorPOV = operatorController.getPOV(0);


    // PRIMARY OPERATOR CONTROL BINDINGS
    if (operatorController.getXButton()) 
    { // X hold is for shooter preset mode
      mechSystem.setIntakeAngle(0.18);
      mechSystem.getIntakeSubsystem().isExtended = true;
    } 
    

    else if (operatorController.getYButton()) 
    { // Y hold is designated as a manual control mode


      if (operatorPOV == 0)
      { // Incrament shooter angle in the positive direction
        mechSystem.incramentShooterAngle(1);
      }
      else if (operatorPOV == 180)
      { // Incrament shooter angle in the negative direction
        mechSystem.incramentShooterAngle(-1);
      }


      if (operatorController.getRightBumper())
      { // Set intake power
        mechSystem.setIntakePower(1);
      }
      else if (operatorController.getLeftBumper())
      {
        mechSystem.setIntakePower(-1);
      }
      else
      {
        mechSystem.setIntakePower(0);
      }

      if (operatorPOV == 270)
      {
        mechSystem.incramentIntakeAngle(0.001);
      }
      else if (operatorPOV == 90)
      {
        mechSystem.incramentIntakeAngle(-0.001);
      }


      mechSystem.setHopperPower(MiscUtils.clamp(-0.3, 0.3, operatorController.getRightTriggerAxis() - operatorController.getLeftTriggerAxis()));


    } 
    

    else if (operatorController.getRightTriggerAxis() > 0.8 &&
      operatorController.getLeftTriggerAxis() > 0.8)
    { // Double bumper hold is designated for climber controls
      

      if (operatorController.getLeftY() > 0.8)
      { // Left climber controls
        mechSystem.incClimberLeft(0.4);
      }
      else if (operatorController.getLeftY() < -0.8)
      {
        mechSystem.incClimberLeft(-0.4);
      }
      else
      {
        mechSystem.incClimberLeft(0);
      }


      if (operatorController.getRightY() > 0.8)
      { // Right climber controls
        mechSystem.incClimberRight(0.4);
      }
      else if (operatorController.getRightY() < -0.8)
      {
        mechSystem.incClimberRight(-0.4);
      }
      else
      {
        mechSystem.incClimberRight(0);
      }


    }


    else if (operatorController.getBButtonPressed()) 
    { // B hold is a wildcard as of writing this comment
      mechSystem.setIntakeAngle(0.06);
      mechSystem.getIntakeSubsystem().isExtended = true;
    } 
    
    
    else 
    { // Default operating mode that allows intake functionality only


      // Stop the shooter
      //mechSystem.setShooterSpeed(0);
      //mechSystem.setShooterPower(0);
      mechSystem.executePreset(0, 0);

      mechSystem.setHopperPower(0);




      if (operatorPOV == 0) 
      { // Bumpers touching speaker preset
        //mechSystem.setShooterAngle(ShooterConstants.speakerPresetPosition);
        mechSystem.executePreset(ShooterConstants.speakerPresetPosition, 3800);
        SmartDashboard.putString("Last Preset", "Bumper-Speaker");
      }


      if (operatorPOV == 270) 
      { // Bumpers touching Amp preset
        //mechSystem.setShooterAngle(ShooterConstants.ampPresetPosition);
        mechSystem.executePreset(ShooterConstants.ampPresetPosition, 1650);
        SmartDashboard.putString("Last Preset", "Amp");
      }


      if (operatorPOV == 180) 
      { // Aligned with chain trap shoot preset
        //mechSystem.setShooterAngle(ShooterConstants.trapPresetPosition);
        mechSystem.executePreset(ShooterConstants.trapPresetPosition, 3800);
        SmartDashboard.putString("Last Preset", "Chain-Trap");
      }

      if (operatorPOV == 90)
      { // Touching alliance boundry from inside alliance zone
        //mechSystem.setShooterAngle(ShooterConstants.redLinePresetPosition);
        mechSystem.executePreset(ShooterConstants.speakerPresetPosition, 4500);
        SmartDashboard.putString("Last Preset", "Alliance-Boundry");
      }



      if (operatorController.getRightStickButton()) {
        mechSystem.setShooterAngleWithOdometry();
      }

      if (operatorController.getRightTriggerAxis() > 0.8 && operatorController.getLeftTriggerAxis() < 0.3) {
        mechSystem.setShooterSpeed(3800);
        //mechSystem.setShooterAngleWithVision();
        // Aimbot jr goes based off of odometry rather than the vision target itself (ALPHA STATE)
        mechSystem.setShooterAngleWithOdometry();
      }



      if (operatorController.getRightBumper()) {
        mechSystem.setHopperPower(1);
      } else if (operatorController.getLeftBumper()) {
        mechSystem.setHopperPower(-1);
      } else {
        mechSystem.setHopperPower(0);
      }

      // Things that once existed on the X preset

      if (operatorController.getRightBumperPressed())
      { // Set intake power
        mechSystem.setIntakePower(1);
        // Set hopper if using preset
        mechSystem.getIntakeSubsystem().startTime = System.currentTimeMillis();
      }
      
      if (operatorController.getLeftBumperPressed())
      {
        mechSystem.setIntakePower(-1);
        if (operatorPOV == -1) {mechSystem.setHopperPower(-1);} else {mechSystem.setHopperPower(0);}
        mechSystem.getIntakeSubsystem().startTime = System.currentTimeMillis();
        floodLights.isReserved = false;
      }

      if ((System.currentTimeMillis() - mechSystem.getIntakeSubsystem().startTime > 50) && 
          mechSystem.getIntakeSubsystem().getOutputCurrent() > 5 && 
          mechSystem.getIntakeSubsystem().firstSpike) {

        mechSystem.getIntakeSubsystem().stopTimer.restart();
        mechSystem.getIntakeSubsystem().firstSpike = false;
        floodLights.isReserved = false;
      } else if (mechSystem.getIntakeSubsystem().getOutputCurrent() < 5) {
        mechSystem.getIntakeSubsystem().firstSpike = true;
        floodLights.isReserved = false;
      }
      
      if (operatorController.getRightBumperReleased() || 
          operatorController.getLeftBumperReleased() ||
          ( mechSystem.getIntakeSubsystem().getOutputCurrent() > 5 && 
            !mechSystem.getIntakeSubsystem().isExtended && 
            mechSystem.getIntakeSubsystem().stopTimer.hasElapsed(0.1) &&
            !mechSystem.getIntakeSubsystem().firstSpike))
      {
        mechSystem.setIntakePower(0);
        if (operatorController.getRightBumper()) {
          floodLights.isReserved = true;
          floodLights.storedPreset("solid-pink");
          SmartDashboard.putString("Lights", "Pink");
        }

        floodLights.isReserved = false;
      }


      if (operatorController.getAButtonPressed() || operatorController.getAButtonReleased())
      { // Toggle intake angle
        mechSystem.toggleIntakeState();
      }

      // TMP - Set climbers to zero when not in double bumper mode.
      mechSystem.setClimberLeft(140);
      mechSystem.setClimberRight(140);


    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
