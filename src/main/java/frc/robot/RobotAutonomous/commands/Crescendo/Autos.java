// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotAutonomous.commands.Crescendo;

import java.util.List;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.RobotMechanisms.Crescendo.MechanismConstants.ShooterConstants;
import frc.robot.RobotMechanisms.Crescendo.Subsystems.VisionSubsystemLegacy;
import frc.robot.BaseConstants.AutoConstants;
import frc.robot.BaseConstants.DriveConstants;
import frc.robot.RobotChassis.Subsystems.SwerveChassisSubsystem;
import frc.robot.RobotMechanisms.Crescendo.MechanismSubsystem;
import frc.robot.RobotMechanisms.Crescendo.Commands.DriveWithVisionLegacyAuto;

/** This file contains legacy autos */
public class Autos {

  // THESE AUTOS ARE VERY MUCH BROKEN AT THE MOMENT.
  // IT IS RECOMMENDED TO REBUILD AUTOS IN PATH PLANNER.


  // General Operations
  public static TrajectoryConfig getTrajectoryConfig(boolean reversed) {
    TrajectoryConfig config = new TrajectoryConfig(
      AutoConstants.kMaxSpeedMetersPerSecond,
      AutoConstants.kMaxAccelerationMetersPerSecondSquared)
      .setKinematics(DriveConstants.kDriveKinematics);

    config.setReversed(reversed);

    return config;
  }

  public static ProfiledPIDController getThetaController() {
    ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    return thetaController;
  }


  public static Pose2d getPose(double x, double y, double rotation) {
    return new Pose2d(new Translation2d(x, y), new Rotation2d(rotation));
  }

  // General Commands
  public static Command getStopCommand(SwerveChassisSubsystem drive) {
    return Commands.runOnce(() -> {System.out.println("STOP"); drive.drive(new ChassisSpeeds(0, 0, 0));}, drive);
  }

  public static SwerveControllerCommand driveToLocationCommand(Pose2d startPose, Pose2d targetPose, boolean reversed, SwerveChassisSubsystem drive) {
    edu.wpi.first.math.trajectory.Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      startPose,
      List.of(),
      targetPose,
      getTrajectoryConfig(reversed)
    );


    return new SwerveControllerCommand(
      trajectory,
      drive::getPose, // Functional interface to feed supplier
      DriveConstants.kDriveKinematics,

      // Position controllers
      new PIDController(AutoConstants.kPXController, 0, 0),
      new PIDController(AutoConstants.kPYController, 0, 0),
      getThetaController(),
      drive::setModuleStates,
      drive
    );
  }

  // Components of the relative autos
  public static Command runShooterSpeakerPreset(MechanismSubsystem mechSystem) {
    return Commands.runOnce(() -> { mechSystem.executePreset(ShooterConstants.speakerPresetPosition, 4000); }, mechSystem);
  }

  public static Command aimShooterWithVision(MechanismSubsystem mechSystem) {
    return Commands.runOnce(() -> {mechSystem.setShooterAngleWithVision();}, mechSystem);
  }

  public static Command setShooterProfile(double angle, double speed, MechanismSubsystem mechSystem) {
    return Commands.runOnce(() -> { mechSystem.executePreset(angle, speed);}, mechSystem);
  }

  public static Command aimShooterWithDistance(double distance, MechanismSubsystem mechSystem) {
    return Commands.runOnce(() -> {mechSystem.setShooterAngleFromDistance(distance);}, mechSystem);
  }

  // Fix This
  public static Command rotateTowardsSpeaker(SwerveChassisSubsystem drive) {
    return Commands.runOnce(() -> {new DriveWithVisionLegacyAuto(0, 0, 8, drive);}, drive);
  }

  public static Command intakeAngle(double angle, MechanismSubsystem mechSystem) {
    return Commands.runOnce(() -> { mechSystem.setIntakeAngle(angle);}, mechSystem);
  }

  public static Command intakePower(double power, MechanismSubsystem mechSystem) {
    return Commands.runOnce(() -> { mechSystem.setIntakePower(power);}, mechSystem);
  }

  public static Command hopperPower(double power, MechanismSubsystem mechSystem) {
    return Commands.runOnce(() -> { mechSystem.setHopperPower(power);}, mechSystem);
  }

  public static Command intakeAndHopperPower(double powerH, double powerI, MechanismSubsystem mechSystem) {
    return Commands.runOnce(() -> { mechSystem.setHopperPower(powerH); mechSystem.setIntakePower(powerI);}, mechSystem);
  }

  public static Command resetOdometry(SwerveChassisSubsystem drive) {
    return Commands.runOnce(() -> {System.out.println("RESET"); drive.resetOdometry(getPose(0, 0, 0));}, drive);
  }

  // Combined commands
  public static Command shoot(boolean dontStop, double triggerSpeed, MechanismSubsystem mechSystem) {
    BooleanSupplier conditions = () -> {

      boolean upToSpeed = mechSystem.getShooterSpeed() >= triggerSpeed;
      boolean intakeFlat = mechSystem.getIntakePos() <= 0.205;

      if (upToSpeed && intakeFlat) {
        return true;
      } else {
        return false;
      }

    };

    return intakeAngle(0.18, mechSystem)
      .andThen(Commands.waitUntil(conditions))
      .andThen(intakeAndHopperPower(1, 1, mechSystem))
      .andThen(Commands.waitSeconds(1))
      .andThen(intakeAndHopperPower(0, 0, mechSystem))
      .andThen(setShooterProfile(0, dontStop ? triggerSpeed + 200 : 0, mechSystem));
  }

  public static Command aimAndShootWithVision(MechanismSubsystem mechSystem, SwerveChassisSubsystem drive) {
    return getStopCommand(drive)
    .andThen(
      new ParallelCommandGroup(
        rotateTowardsSpeaker(drive).repeatedly().until(() -> Math.abs(mechSystem.getVisionObject().getAprilTagX(0) - 300) < 20),
        Commands.runOnce(() -> mechSystem.setShooterAngleWithVision(), mechSystem).repeatedly().withTimeout(2)
      )
    )
    .andThen(getStopCommand(drive))
    .andThen(shoot(true, 3800, mechSystem))
    .andThen(intakeAngle(0.16, mechSystem));
  }

  public static Command intakeWithCurrentThreshold(MechanismSubsystem mechSystem) {
    BooleanSupplier hitCurrentThreshold = () -> {
      return mechSystem.getIntakeSubsystem().getOutputCurrent() > 5;
    };

    return Commands.waitUntil(hitCurrentThreshold).withTimeout(5)
      .andThen(Commands.waitSeconds(0.2))
      .andThen(intakePower(0, mechSystem))
      .andThen(intakeAngle(0.16, mechSystem));
  }

  public static Command leave(SwerveChassisSubsystem drive) {
    
    return resetOdometry(drive)
      .andThen(driveToLocationCommand(getPose(0, 0, 0), getPose(1.5, 0, 0), false, drive))
      .andThen(Commands.runOnce(() -> {drive.drive(new ChassisSpeeds(0, 0, 0));}, drive));
  }

  // DREW PIECE CENTER
  public static Command drewPiece(MechanismSubsystem mechSystem, SwerveChassisSubsystem drive) {
    return 
      setShooterProfile(0, 4000, mechSystem)
      .andThen(driveToLocationCommand(getPose(0, 0, 0), 
        getPose(0.6, 0, 0), false, drive))
      .andThen(getStopCommand(drive))
      .andThen(aimAndShootWithVision(mechSystem, drive))
      .andThen(Commands.runOnce(() -> drive.drive(new ChassisSpeeds(0, 0, 0))).repeatedly().withTimeout(2))
      .andThen(getStopCommand(drive))
      .andThen(intakeAngle(0.35, mechSystem))
      .andThen(intakePower(1, mechSystem))
      .andThen(
        new ParallelCommandGroup(
          driveToLocationCommand(getPose(0.6, 0, 0), getPose(1, 0, 0), false, drive),
          Commands.waitSeconds(0.5).andThen(intakeWithCurrentThreshold(mechSystem))
        )
      )
      .andThen(getStopCommand(drive))
      .andThen(aimAndShootWithVision(mechSystem, drive))
      .andThen(setShooterProfile(0, 0, mechSystem));
  }

  // Methods for path planner center auto

  // Aim the shooter via the robots odometry (OPTIONAL: Spin Up the Shooter To Specified Speed)
  public static Command aimShooterWithOdometry(MechanismSubsystem mechSystem, double shooterSpeedRPM) {
    return Commands.runOnce(() -> {
      mechSystem.setShooterAngleWithOdometry();
      if(shooterSpeedRPM > 0) {
        mechSystem.setShooterSpeed(shooterSpeedRPM);
      }
    }, mechSystem);
  }

  // Aim the shooter via the robots odometry (OPTIONAL: Spin Up the Shooter To Specified Speed)
  public static Command aimAndShootWhenReady(MechanismSubsystem mechSystem, double shooterSpeedRPM, double triggerSpeed, boolean dontStop) {
    // aimShooterWithOdometry(Subsystem, SpeedToSpinUpTo)   shoot(dontStop, SpeedThatTriggersTheShoot, Subsystem)
    return new ParallelRaceGroup(aimShooterWithOdometry(mechSystem, shooterSpeedRPM).repeatedly(), shoot(dontStop, triggerSpeed, mechSystem));
  }


  public static void registerCommandsCrescendo(SwerveChassisSubsystem chassis, MechanismSubsystem mechSystem, VisionSubsystemLegacy vision) {
    /* --------------------------------------------------------
     * The Following Code Is A Path Planner Implimentation Of
     * An Auto That Aims And Shoots It's Preloaded Game Piece
     * Based Off Of It's Vision Assisted, Internal Odometry.
     * -------------------------------------------------------*/
    // Score pre-loaded note: Stage 1
    NamedCommands.registerCommand("setShooterRPM3800", new RunCommand(() -> {mechSystem.setShooterSpeed(3800);}, mechSystem));
    NamedCommands.registerCommand("aimAtSpeaker", aimShooterWithOdometry(mechSystem, 0).repeatedly());
    // Score pre-loaded note: Stage 2
    NamedCommands.registerCommand("levelIntake", intakeAngle(0.16, mechSystem));
    NamedCommands.registerCommand("shootWhenReady", shoot(false, 3600, mechSystem));
    NamedCommands.registerCommand("stopShooter", new RunCommand(() -> {mechSystem.setShooterSpeed(0);}, mechSystem));
    NamedCommands.registerCommand("aimAndShootWhenReady", aimAndShootWhenReady(mechSystem, 3800, 3600, false));
  }


}
