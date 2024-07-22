// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotVision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotVision.VisionConstants.PhotonConstants;
import frc.robot.RobotVision.VisionConstants.PositionConstants;

// This class is responsible for processing the results
// of the vision data streamed over via from the Pi

public class VisionSubsystem extends SubsystemBase {

  // THIS CLASS IS IN ITS INFANCY, AND IS SUBJECT TO LARGE CHANGES

  // Create camera objects for each camera on the robot.
  // The camera name should be the name of the Network Table
  // that contains the camera stream.
  public PhotonCamera camera1 = new PhotonCamera(PhotonConstants.cameraName1);

  // Create a photon pose estimator for april tag, field pose estimation
  private PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(
    PositionConstants.tagPositions, 
    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
    camera1, 
    PositionConstants.robotToCam);

  // Create a refrence to the global pose estimator in the chassis subsystem
  //SwerveDrivePoseEstimator globalOdometry = SubsystemContainer.getSingletonInstance().getChassis().swervePoseEstimator;

  // Nothing is needed in the constructor at the moment, so thats why it's empty.
  public VisionSubsystem() {/* Nothing to init here! */}


  /* ----------------------------------------------------
   * Fundamental methods for basic camera functions
   * -------------------------------------------------- */

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


  /* ----------------------------------------------------
   * Utilities for common vision processing techniques
   * -------------------------------------------------- */

  public Optional<EstimatedRobotPose> getPoseEstimate() {
    return poseEstimator.update();
  }

  // Returns yaw difference to a given pose on the field
  public Rotation2d getYawToPose(Pose2d currentPose, Pose2d targetPose) {
    return PhotonUtils.getYawToPose(currentPose, targetPose);
  }

  // Returns yaw difference to a given tag on the field
  public Rotation2d getYawToTag(Pose2d currentPose, int tagId) {
    Pose2d tagPose; // The tag poase shouldnt be null, but in the case that it is, well pass the problem onto the caller
    try { tagPose = PositionConstants.tagPositions.getTagPose(tagId).get().toPose2d(); } catch (Exception e) { return null; }
    return PhotonUtils.getYawToPose(currentPose, tagPose);
  }

  // Returns a processed transform based on all the tag positions in the frame from the coprocessor.
  public Transform3d getMultiTagTarget(PhotonCamera photonCamera) {
    PNPResult data = getLatest(photonCamera).getMultiTagResult().estimatedPose;
    if (data.isPresent) {
      return data.best;
    } else {
      return null;
    }
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

  // A function to apply the vision pose to the global estimated robot pose
  public Pose2d getVisionOdometryEstimate() {
    Optional<EstimatedRobotPose> estimate = getPoseEstimate();
    if (estimate.isPresent()) {
      Pose3d estimatedRobotPose = estimate.get().estimatedPose;
      return estimatedRobotPose.toPose2d();
    } else {
      return null;
    }
  }

  @Override
  public void periodic() {
    /*updateRobotOdometryWithVision();*/
  }
}
