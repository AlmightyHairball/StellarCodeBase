// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotVision;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotVision.VisionConstants.PhotonConstants;

// This class is responsible for processing the results
// of the vision data streamed over via from the Pi

public class VisionSubsystem extends SubsystemBase {

  // THIS CLASS IS IN ITS INFANCY, AND IS SUBJECT TO LARGE CHANGES

  // Create camera objects for each camera on the robot.
  // The camera name should be the name of the Network Table
  // that contains the camera stream.
  private PhotonCamera camera = new PhotonCamera(PhotonConstants.cameraName1);

  public VisionSubsystem() {
    // Send a notifier when camera is online
    if (this.camera.isConnected()) {System.out.println("CAMERA CONNECTION SUCCESSFUL!");}
  }

  // Returns the latest information from photonvision
  public PhotonPipelineResult getLatest() {
    return this.camera.getLatestResult();
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

  public double[] getBestTagData() {
    double[] data = new double[4];
    data[0] = this.getBestVisibleTarget().getFiducialId();
    data[1] = this.getBestVisibleTarget().getPitch();
    data[2] = this.getBestVisibleTarget().getYaw();
    data[3] = this.getBestVisibleTarget().getSkew();
    data[4] = this.getBestVisibleTarget().getArea();

    return data;
  }

  @Override
  public void periodic() {
    
  }
}
