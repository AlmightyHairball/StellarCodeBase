// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotMechanisms.Crescendo.Subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BaseConstants.ModuleConstants;
import frc.robot.RobotMechanisms.Crescendo.MechanismConstants.IntakeConstants;
import frc.robot.RobotUtilities.MiscUtils;

// This subsystem is for controlling the intake of the
// 2024 robot (Epic).

public class IntakeSubsystem extends SubsystemBase {

    // Declare Motors, encoders, and Position Controllers
    private final CANSparkMax intakeDrive;
    private final CANSparkMax intakeAngle;
    private final AbsoluteEncoder intakeAngleEncoder;
    private double lastAngle;

    // Using SparkPIDController instead of a generic one in order
    // to execute PID operatons on the angle motor controller.
    private final SparkPIDController angleController;

    // A status variable for intake toggle functionality
    public boolean isExtended;
    public long startTime;
    public Timer stopTimer = new Timer();
    public boolean firstSpike = true;

  public IntakeSubsystem() {

        // Intake toggle starts in the false case
        isExtended = true;
        lastAngle = 0.6;

        // Define Spark Motors
        intakeDrive = new CANSparkMax(IntakeConstants.intakeDriveControllerId, MotorType.kBrushless);
        intakeAngle = new CANSparkMax(IntakeConstants.intakeAngleControllerId, MotorType.kBrushless);

        // Reset motor controllers to factory settings
        intakeDrive.restoreFactoryDefaults();

        // Were disabling the factory reset so we dont lose out absolute encoder software offeset.
        //intakeAngle.restoreFactoryDefaults();

        // Set current limits
        intakeDrive.setSmartCurrentLimit(30, 30);
        intakeAngle.setSmartCurrentLimit(30, 30);

        // Grab the encoders of the motors
        intakeAngleEncoder = intakeAngle.getAbsoluteEncoder(Type.kDutyCycle);

        // Define Position Controller for the Angle Motor
        angleController = intakeAngle.getPIDController();
        angleController.setFeedbackDevice(intakeAngleEncoder);

        // Setting intial PID Values for the Angle
        angleController.setP(IntakeConstants.intakeAngleP);
        angleController.setI(IntakeConstants.intakeAngleI);
        angleController.setD(IntakeConstants.intakeAngleD);
        angleController.setFF(0);
        angleController.setOutputRange(-1, 1);

        // Making sure our intake doesent go through our robot
        angleController.setPositionPIDWrappingEnabled(true);
        angleController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
        angleController.setPositionPIDWrappingMaxInput(1);

        // Flash motor configuration to the controllers
        intakeDrive.burnFlash();
        intakeAngle.burnFlash();

        // Setting a starting position for the intake
        angleController.setReference(0.05, CANSparkMax.ControlType.kPosition);

  }

// Intake drive motor setters.
public void setDriveSpeed(double speed) {
    intakeDrive.set(speed);
}

// This only stops the motor when its in drive mode
public void stopDrive() {
    intakeDrive.set(0);
}

// Intake drive motor getter (power mode only)
public double getDriveSpeed() {
    return intakeDrive.get();
}

// Angle Encoder operation
public double getAngleEncoderPos() {
    return intakeAngleEncoder.getPosition();
}

public double getOutputCurrent() {
    return intakeDrive.getOutputCurrent();
}

// Angle Controller Setters
public void setTargetAngle(double angleDegrees) {
    /* 
    double newDown = 0; // TODO: FIX ME
    double newMid = 0; //TODO: FIX ME
    double newHigh = 0;

    if (angleDegrees > Constants.IntakeConstants.intakeMaxAngle - 0.1) {
        angleController.setReference(newDown, ControlType.kPosition);
    }*/
    
    angleController.setReference(MiscUtils.clamp(IntakeConstants.intakeMinAngle, IntakeConstants.intakeMaxAngle, angleDegrees), ControlType.kPosition);
}

// Incrament intake angle
public void incramentAngle(double rotations) {
    lastAngle = MiscUtils.clamp(IntakeConstants.intakeMinAngle, IntakeConstants.intakeMaxAngle, lastAngle + rotations);
    this.setTargetAngle(lastAngle);
}



// Switch the state up and down
public void toggleState() {
    if (isExtended) {
        this.setTargetAngle(0.35);
    } else {
        this.setTargetAngle(0.18);
    }
    // Update our extension status
    isExtended = !isExtended;
}
    

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
