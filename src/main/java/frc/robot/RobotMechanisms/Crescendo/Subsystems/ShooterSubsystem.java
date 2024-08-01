// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotMechanisms.Crescendo.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SubsystemContainer;
import frc.robot.RobotMechanisms.Crescendo.MechanismConstants.ShooterConstants;
import frc.robot.RobotUtilities.MiscUtils;
import frc.robot.RobotVision.VisionSubsystem;

public class ShooterSubsystem extends SubsystemBase {

    // Declare motors, encoders, PID controllers, etc
    private final CANSparkMax shooterDriveController;
    private final CANSparkMax shooterAngleController;

    private final RelativeEncoder shooterAngleEncoder;
    private final RelativeEncoder shooterDriveEncoder;

    private final SparkPIDController shooterAnglePIDController;
    private final SparkPIDController shooterDrivePIDController;

    private VisionSubsystemLegacy vision;

    // Set any specific params to the shooter
    private double lastDriveSpeed = 0;
    private double lastAngle = 0;

    private final double SPEAKER_HEIGHT = 2.047;
    private final double HEIGHT_OFFSET = 0;
    
    private final double APRIL_TAG_TO_FLOOR = 1.45;
    private final double CAMERA_TO_FLOOR = 0.37;
    private final double Z_OFFSET = 0.25; // meters

    private final double ANGLE_OFFSET = 7.5;

    private final double DEGREES_TO_ROTATIONS = 3;

    NetworkTableInstance nt;
    NetworkTable table;

    public DoublePublisher anglePub;
    public DoublePublisher speedPub;

    // Some sort of linear filter for the vision
    LinearFilter zFilter = LinearFilter.singlePoleIIR(0.5, 0.02);

  public ShooterSubsystem(VisionSubsystemLegacy vision) {

        nt = NetworkTableInstance.getDefault();
        table = nt.getTable("SmartDashboard");

        anglePub = table.getDoubleTopic("shooterAngle2").publish();
        speedPub = table.getDoubleTopic("shooterSpeed").publish();

        anglePub.set(0);
        speedPub.set(0);

        // Define Motors
        shooterDriveController = new CANSparkMax(ShooterConstants.shooterDriveControllerID, MotorType.kBrushless);
        shooterAngleController = new CANSparkMax(ShooterConstants.shooterAngleControllerID, MotorType.kBrushless);

        // Reset motor controllers to factory settings
        shooterDriveController.restoreFactoryDefaults();
        shooterAngleController.restoreFactoryDefaults();

        // Set current
        shooterDriveController.setSmartCurrentLimit(40);
        shooterAngleController.setSmartCurrentLimit(30);

        // Get the angle controller encoder
        shooterAngleEncoder = shooterAngleController.getEncoder();
        shooterDriveEncoder = shooterDriveController.getEncoder();

        // Get the angle PID controller
        shooterAnglePIDController = shooterAngleController.getPIDController();
        shooterDrivePIDController = shooterDriveController.getPIDController();

        // Set Angle motor PID values
        shooterAnglePIDController.setP(ShooterConstants.shooterAngleP);
        shooterAnglePIDController.setI(ShooterConstants.shooterAngleI);
        shooterAnglePIDController.setD(ShooterConstants.shooterAngleD);

        shooterDrivePIDController.setP(ShooterConstants.shooterDriveP);
        shooterDrivePIDController.setI(ShooterConstants.shooterDriveI);
        shooterDrivePIDController.setD(ShooterConstants.shooterDriveD);
        shooterDrivePIDController.setIZone(ShooterConstants.shooterDriveIZone);

        // Set PID Feedback Device
        shooterAnglePIDController.setFeedbackDevice(shooterAngleEncoder);
        shooterDrivePIDController.setFeedbackDevice(shooterDriveEncoder);
        shooterAngleController.setInverted(true);
        shooterDriveController.setInverted(true);

        // Flash motor configuration to the controllers
        shooterDriveController.burnFlash();
        shooterAngleController.burnFlash();

        this.vision = vision;

  }

  // Drive motor getters and setters
  public void setDrivePower(double speedPower) {
      shooterDriveController.set(speedPower);
  }

  public void stopDriveMotors() {
      //shooterDriveController.set(0);
  }

  public double getDrivePower() {
      return shooterDriveController.get();
  }

  // Shooter encoder operations
  public void resetAngleEncoder() {
      shooterAngleEncoder.setPosition(0);
  }

  public void setAngleEncoder(double position) {
      shooterAngleEncoder.setPosition(position);
  }

  public double getAngleEncoderPos() {
      return shooterAngleEncoder.getPosition();
  }

  // Driver velocity control setters
  public void setDriveSpeed(double speedRPMs) { // Dividing to adjust for gearing
      shooterDrivePIDController.setReference(speedRPMs, ControlType.kVelocity);
  }

  public void incramentDriveSpeed(double speedRPMs) {
      lastDriveSpeed = MiscUtils.clamp(-300, 300, lastDriveSpeed + (speedRPMs/50));
      this.setDriveSpeed(lastDriveSpeed);

      this.shooterDriveController.set(1);
  }

  public void resetDriveSpeed() {
      lastDriveSpeed = 0;
      this.setDriveSpeed(0);
  }

  // Angle position control setters
  public void setTargetAngle(double angleRotations) {

      if (Double.isNaN(angleRotations)) {
          return;
      }
      shooterAnglePIDController.setReference(MiscUtils.clamp(ShooterConstants.shooterMinAngle, ShooterConstants.shooterMaxAngle, angleRotations), CANSparkMax.ControlType.kPosition);
  }

  public void setTargetAngleDegrees(double degrees) {
      this.setTargetAngle((degrees - 34) * DEGREES_TO_ROTATIONS);
  }

  public void setVisionAngle() {
      double legacyVisionSource = vision.getAprilTagZ(0);
      double dSquared = Math.pow(legacyVisionSource, 2); // Where d = distance from april tag to camera (refrenced as apriltag z)
      double nSquared = Math.pow(this.APRIL_TAG_TO_FLOOR - this.CAMERA_TO_FLOOR, 2); // self explanatory (if it wasn't already obvious)
      double z = Math.sqrt(dSquared - nSquared); // distance from the robot to the front face of the speaker
      this.setAngleFromDistance(z);
  }

  // Aims the shooter according to the position of a tag in the robots odometry
  public void setAngleWithOdometry() {
    // Gather odometry info for calculation
    VisionSubsystem visionObject = SubsystemContainer.getSingletonInstance().getVisionObject();
    Pose2d estimatedRobotPosition = SubsystemContainer.getSingletonInstance().getChassis().getGlobalPose();
    int targetTag = MiscUtils.isRedAlliance().getAsBoolean() ? 4:7;

    // distanceToTarget serves as "a" and targetHeight "b" in our pythagorean equation.
    double targetHeight = 3.05; // In meters
    double distanceToTarget = visionObject.getDistanceToTag(estimatedRobotPosition, targetTag);

    // Define shooter transform
    double xTranslate = -0.127; // shooter front or back offset horizontal
    double zTranslate = 0.43; // Shooter pivot height (from ground)
    double revisedTargetHeight = targetHeight - zTranslate;
    double revisedDistanceToTarget = distanceToTarget + xTranslate; // add because robot faces forward when shooting
    SmartDashboard.putNumber("TargetHeight", revisedTargetHeight);
    SmartDashboard.putNumber("TargetDistance", revisedDistanceToTarget);

    double thetaVal = Math.atan(revisedTargetHeight / revisedDistanceToTarget);
    double thetaToDeg = Math.toDegrees(thetaVal);
    SmartDashboard.putNumber("thetaDegrees", thetaToDeg);
    this.setTargetAngleDegrees(thetaToDeg);
  }

  public void setAngleFromDistance(double distance) {
      double angle = Math.toDegrees(Math.atan2(SPEAKER_HEIGHT - CAMERA_TO_FLOOR + HEIGHT_OFFSET, distance - Z_OFFSET)) + ANGLE_OFFSET;
      
      anglePub.set(angle);
      speedPub.set(shooterDriveEncoder.getVelocity());

      this.setTargetAngleDegrees(angle);
  }

  public void incrementAngle(double rotations) {
      lastAngle = MiscUtils.clamp(ShooterConstants.shooterMinAngle, ShooterConstants.shooterMaxAngle, lastAngle + rotations);
      this.setTargetAngle(lastAngle);
  }

  public void executePreset(double position, double speed) {
      this.setTargetAngle(position);
      this.setDriveSpeed(speed);
  }

  public double getShooterSpeed() {
      return this.shooterDriveEncoder.getVelocity();
  }

  public double getShooterTemp() {
      return this.shooterDriveController.getMotorTemperature();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
