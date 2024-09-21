// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotMechanisms.Crescendo;

import edu.wpi.first.math.controller.PIDController;

/** Constants specific to the 2024 Crecendo mechanisms. */
public final class MechanismConstants {
    
    // Climber Constants
    public static final class ClimberConstants {

        // Climber motor can ID's
        public static final int climberLeftCANID = 39;
        public static final int climberRightCANID = 40;

        public static final double maxEncoderTicks = 318;
        public static final double halfEncoderTicks = 150;

        public static final double climberP = 0.025;
        public static final double climberI = 0.0005;
        public static final double climberD = 0;

    }

    // Intake Constants
    public static final class IntakeConstants {

        // Motor controller Id's
        public static final int intakeDriveControllerId = 58;
        public static final int intakeAngleControllerId = 54;

        // PID values for the intake angle
        public static final double intakeAngleP = 2;
        public static final double intakeAngleI = 0;
        public static final double intakeAngleD = 0;

        // Min and Max rotational angles
        public static final double intakeMaxAngle = 0.35;
        public static final double intakeMinAngle = 0.05;

    }

    // Shooter Constants
    public static final class ShooterConstants {

        // Motor contrller Id's
        public static final int shooterDriveControllerID = 13;
        public static final int shooterAngleControllerID = 12;

        // PID values for the shooter angle (Used for AimBot Jr)
        public static final double shooterAngleP = 0.02;
        public static final double shooterAngleI = 0;
        public static final double shooterAngleD = 0;

        // PID values for the shooter drive motor (velocity control)
        public static final double shooterDriveP = 0.001;
        public static final double shooterDriveI = 0.000001;
        public static final double shooterDriveD = 0.001;
        public static final double shooterDriveIZone = 1000;

        // Min and Max rotational angle in encoder ticks
        public static final double shooterMaxAngle = 200;
        public static final double shooterMinAngle = 0;

        public static double speakerPresetPosition = 130;// Ideal speed is (3800RPM)
        public static double ampPresetPosition = 170; // Ideal speed is (1600RPM)
        public static double redLinePresetPosition = 40; // Probably doesent work (4500RPM)
        public static double trapPresetPosition = 50; // Ideal speed is (?RPM)

        public static double presetRPMs = 4500;

    }

    // Hopper Constants
    public static final class HopperConstants {
        public static final int HopperControllerID = 11;
    }

    public static class MiscConstants {
        public static final int lightControllerPort = 9;

        // This is a place to store objects that need to be persistent
        public static double stellarControllerAngleOffset = 0;
        public final static PIDController aimBot = new PIDController(0.02, 0, 0);
    }

}
