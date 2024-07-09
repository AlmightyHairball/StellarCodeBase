// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotVision;

import edu.wpi.first.math.util.Units;

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
        public static final String cameraName2 = "StellarVision2";
        public static final String cameraName3 = "StellarVision3";

    }

    public static final class PositionConstants {

        // Camera 1 positional data
        public static final double camera1HeightMeters = Units.inchesToMeters(0);
        public static final double camera1PitchRadians = Units.degreesToRadians(0);

        // Camera 1 positional data
        public static final double camera2HeightMeters = Units.inchesToMeters(0);
        public static final double camera2PitchRadians = Units.degreesToRadians(0);

        // Camera 1 positional data
        public static final double camera3HeightMeters = Units.inchesToMeters(0);
        public static final double camera3PitchRadians = Units.degreesToRadians(0);

    }

}
