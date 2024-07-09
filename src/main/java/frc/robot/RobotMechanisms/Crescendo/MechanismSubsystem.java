// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotMechanisms.Crescendo;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMechanisms.Crescendo.Subsystems.ClimberSubsystem;
import frc.robot.RobotMechanisms.Crescendo.Subsystems.HopperSubsystem;
import frc.robot.RobotMechanisms.Crescendo.Subsystems.IntakeSubsystem;
import frc.robot.RobotMechanisms.Crescendo.Subsystems.ShooterSubsystem;
import frc.robot.RobotMechanisms.Crescendo.Subsystems.VisionSubsystemLegacy;

public class MechanismSubsystem extends SubsystemBase {

  // Create mechanism objects from their respective subsystems
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final ClimberSubsystem climber = new ClimberSubsystem();
  private final HopperSubsystem hopper = new HopperSubsystem();
  private ShooterSubsystem shooter;

  // Define a vision object (Legacy)
  private static VisionSubsystemLegacy vision;


  public MechanismSubsystem() {}

  /* This class is utilizing the singleton pattern
   * because there should be no other SubsystemContainer 
   * other than the one instance that this class hands out. */

  // Create singleton instance
  private static MechanismSubsystem instance;

  // Create method for getting singleton instance.
  // If theres no instance, create one, then return the instance.
  public static MechanismSubsystem getSingletonInstance() {
    if (instance == null) {
      instance = new MechanismSubsystem();
      instance.init(new VisionSubsystemLegacy());
    }
    return instance;
  }

  // Used for initiating the singleton subsystem
  private void init(VisionSubsystemLegacy vision) {
    // Make the passed vision subsystem availible in class scope
    this.vision = vision;
    // Create the ShooterSubsystem with the passed vision subsystem
    this.shooter = new ShooterSubsystem(vision);
  }

  // Vision Methods
  public VisionSubsystemLegacy getVisionObject() {
    return vision;
  }


  // Intake Methods
  public void setIntakePower(double xSpeed) {
    intake.setDriveSpeed(xSpeed);
  }

  public void setIntakeAngle(double angle) {
    intake.setTargetAngle(angle);
  }

  public void toggleIntakeState() {
    intake.toggleState();
  }

  public void incramentIntakeAngle(double rotations) {
    intake.incramentAngle(rotations);
  }

  public double getIntakePos() {
    return intake.getAngleEncoderPos();
  }

  // DONT DO THIS! Do as I say, not as I do. (I promise this code will not be recycled)
  public IntakeSubsystem getIntakeSubsystem() {
    return intake;
  }


  // Shooter Methods
  public void setShooterPower(double shooterSpeed) {
    shooter.setDrivePower(shooterSpeed);
  }

  public void setShooterAngleFromDistance(double distance) {
    shooter.setAngleFromDistance(distance);
  }

  public void setShooterSpeed(double speedRPMs) {
    shooter.setDriveSpeed(speedRPMs);
  }

  public void incramentShooter(double speedRPMsPerSecond) {
    shooter.incramentDriveSpeed(speedRPMsPerSecond);
  }

  public void incramentShooterAngle(double rotations) {
    shooter.incrementAngle(rotations);
  }

  public void stopShooter() {
    shooter.resetDriveSpeed();
  }

  public void setShooterAngle(double angleRotations) {
    shooter.setTargetAngle(angleRotations);
  }

  public void setShooterAngleWithVision() { // WARNING: MAYBE USE THIS METHOD, IT MAY OR MAY NOT BREAK THE ROBOT?
    shooter.setVisionAngle();
  }

  public void executePreset(double position, double speed) {
    shooter.executePreset(position, speed);
  }

  public double getShooterSpeed() {
    return shooter.getShooterSpeed();
  }


  // Climber Methods
  public double[] getClimberEncoderPositions() {
    return climber.getEncoderValues();
  }

  public void incClimberRight(double speed) {
    climber.incramentPositionRight(speed);
  }

  public void incClimberLeft(double speed) {
    climber.incramentPositionLeft(speed);
  }

  public void setClimberRight(double pos) {
    climber.setClimberRightPosition(pos);
  }

  public void setClimberLeft(double pos) {
    climber.setClimberLeftPosition(pos);
  }


  // Hopper Methods
  public void setHopperPower(double power) {
    hopper.setPower(power);
  }

  public static VisionSubsystemLegacy getVision() {
    return vision;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // DELETE THIS AFTER DEBUGGING!
    SmartDashboard.putNumber("IntakeAngle", intake.getAngleEncoderPos());
    SmartDashboard.putNumber("ShooterAngle", shooter.getAngleEncoderPos());

    SmartDashboard.putNumber("ClimberLeftPosition", climber.getEncoderValues()[0]);
    SmartDashboard.putNumber("ClimberRightPosition", climber.getEncoderValues()[1]);
  }
}
