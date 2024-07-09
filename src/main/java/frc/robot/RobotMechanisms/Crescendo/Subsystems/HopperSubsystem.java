// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotMechanisms.Crescendo.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMechanisms.Crescendo.MechanismConstants.HopperConstants;

// This is the most complex mechanism of the bunch which
// regulates the flow of game pieces to the shooter
// on the 2024 robot (Epic).

public class HopperSubsystem extends SubsystemBase {
  
  // Declare our one "all important motor".
  private final CANSparkMax hopperMotor;

  public HopperSubsystem() {
    // Define our one "all important motor".
    hopperMotor = new CANSparkMax(HopperConstants.HopperControllerID, MotorType.kBrushless);
  }

  public void setPower(double power) {
    // Set our one "all important motor".
    hopperMotor.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
