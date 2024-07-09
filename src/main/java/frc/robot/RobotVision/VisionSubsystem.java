// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotVision;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// This class is responsible for processing the results
// of the vision data streamed over via from the Pi

public class VisionSubsystem extends SubsystemBase {

  // THIS CLASS IS IN ITS INFANCY, AND IS SUBJECT TO LARGE CHANGES

  // Create camera objects for each camera on the robot.
  // The camera name should be the name of the Network Table
  // that contains the camera stream.
  private PhotonCamera stellarVision1 = new PhotonCamera("LifeCam"); // Just a placeholder as of the time of writing this.

  // Get apriltag position data via FIRST provided json file
  //private AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  public VisionSubsystem() {
    if (this.stellarVision1.isConnected()) {System.out.println("WE HAVE CAMERA CONNECTION!");}
  }

  // Returns the latest information from photonvision
  public PhotonPipelineResult getLatest() {
    return this.stellarVision1.getLatestResult();
  }

  // Returns whether photon sees one or more apriltags
  public boolean hasTarget(VisionSubsystem vision) {
    return this.getLatest().hasTargets();
  }

  // Returns a list of targets that photon sees
  public List<PhotonTrackedTarget> getVisibleTargets() {
    return this.getLatest().getTargets();
  }

  // Returns the best target in view of the photon camera
  public PhotonTrackedTarget getBestVisibleTarget() {
    return this.getLatest().getBestTarget();
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
