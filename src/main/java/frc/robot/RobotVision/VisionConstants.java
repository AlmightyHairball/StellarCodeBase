// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotVision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/** Constants file for holding vision id and positioning information */
public class VisionConstants {

    // THE METHOD FOR STORING VISION CONSTANTS HAS NOT BEEN FULLY
    // DETERMINED YET AND WILL MOST LIKELY CHANGE A LOT OVER TIME

    // Store data specific to photonvision
    public static final class PhotonConstants {

        /* These names are the same as the camera name in
         * chosen the photon client for each camera and
         * are identified as so in network tables */ 
        public static final String cameraName1 = "StellarVision1";

    }

    public static final class PositionConstants {

        // Camera 1 position relative to the robot center ([Facing Forward In Center][Half A Meter Forward][Half A Meter Up])
        public static final Transform3d robotToCam = new Transform3d(new Translation3d(0, -0.2, 0.4), new Rotation3d(0, -0.43 * Math.PI, -0.965 * Math.PI));

        // Get apriltag position data via FIRST provided json file
        public static final AprilTagFieldLayout tagPositions = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    }

}
