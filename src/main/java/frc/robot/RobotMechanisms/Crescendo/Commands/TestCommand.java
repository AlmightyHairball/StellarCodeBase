// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotMechanisms.Crescendo.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotChassis.Subsystems.SwerveChassisSubsystem;

public class TestCommand extends Command {
  /** Creates a new TestCommand. */
  public TestCommand(SwerveChassisSubsystem chassis) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Test Command Output");
    isFinished();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
