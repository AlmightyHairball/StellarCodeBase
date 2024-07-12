// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotVision;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotVision.VisionConstants.PhotonConstants;

// This class is responsible for processing the results
// of the vision data streamed over via from the Pi

public class VisionSubsystem extends SubsystemBase {

  // THIS CLASS IS IN ITS INFANCY, AND IS SUBJECT TO LARGE CHANGES

  // Create camera objects for each camera on the robot.
  // The camera name should be the name of the Network Table
  // that contains the camera stream.
  private PhotonCamera camera1 = new PhotonCamera(PhotonConstants.cameraName1);

  public VisionSubsystem() {
    // Send a notifier when camera is online
    if (this.camera1.isConnected()) {System.out.println("CAMERA_1 CONNECTION SUCCESSFUL!");}
  }

  // Returns the latest information from photonvision
  public PhotonPipelineResult getLatest(PhotonCamera photonCamera) {
    return photonCamera.getLatestResult();
  }

  // Returns whether photon sees one or more apriltags
  public boolean hasTarget(PhotonCamera photonCamera) {
    return getLatest(photonCamera).hasTargets();
  }

  // Returns a list of targets that photon sees
  public List<PhotonTrackedTarget> getVisibleTargets(PhotonCamera photonCamera) {
    return getLatest(photonCamera).getTargets();
  }

  // Returns the best target in view of the photon camera
  public PhotonTrackedTarget getBestVisibleTarget(PhotonCamera photonCamera) {
    return getLatest(photonCamera).getBestTarget();
  }

  // Grab specific target based off of ID
  public PhotonTrackedTarget getSpecifiedTarget(PhotonCamera photonCamera, int apriltagId) {
      List<PhotonTrackedTarget> tags = getVisibleTargets(photonCamera);
      for (PhotonTrackedTarget tag : tags) {
        if (tag.getFiducialId() == apriltagId) {
          return tag;
        }
      }
      return null;
  }

  // A convenience function to compile the data of the best
  // tag in view into a single array.
  public double[] getBestTagData(PhotonCamera photonCamera) {
    double[] data = new double[6];
    if (photonCamera.getLatestResult().hasTargets()) {
      data[0] = getBestVisibleTarget(photonCamera).getFiducialId();
      data[1] = getBestVisibleTarget(photonCamera).getPitch();
      data[2] = getBestVisibleTarget(photonCamera).getYaw();
      data[3] = getBestVisibleTarget(photonCamera).getSkew();
      data[4] = getBestVisibleTarget(photonCamera).getArea();
    }
    data[5] = photonCamera.getLatestResult().hasTargets() ? 1:0;
    return data;
  }

  // A convenience function to compile the data of the specified
  // tag in view into a single array.
  public double[] getSpecifiedTagData(PhotonCamera photonCamera, int tagId) {
    double[] data = new double[6];
    PhotonTrackedTarget target = getSpecifiedTarget(photonCamera, tagId);
    if (photonCamera.getLatestResult().hasTargets() & target != null) {
      data[0] = getSpecifiedTarget(photonCamera, tagId).getFiducialId();
      data[1] = getSpecifiedTarget(photonCamera, tagId).getPitch();
      data[2] = getSpecifiedTarget(photonCamera, tagId).getYaw();
      data[3] = getSpecifiedTarget(photonCamera, tagId).getSkew();
      data[4] = getSpecifiedTarget(photonCamera, tagId).getArea();
    }
    data[5] = photonCamera.getLatestResult().hasTargets() ? 1:0;
    return data;
  }

  @Override
  public void periodic() {

      double[] visionData = getSpecifiedTagData(camera1, 7);
      SmartDashboard.putNumber("VisionID", visionData[0]);
      SmartDashboard.putNumber("VisionPitch", visionData[1]);
      SmartDashboard.putNumber("VisionYaw", visionData[2]);
      SmartDashboard.putNumber("VisionSkew", visionData[3]);
      SmartDashboard.putNumber("VisionArea", visionData[4]);
      SmartDashboard.putNumber("VisionTarget", visionData[5]);

  }
}
