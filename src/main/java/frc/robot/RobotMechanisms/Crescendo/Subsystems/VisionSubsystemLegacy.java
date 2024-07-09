// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotMechanisms.Crescendo.Subsystems;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystemLegacy extends SubsystemBase {
  
  // Declare variables needed for NetworkTables
  NetworkTableInstance nt;
  NetworkTable table;

  DoubleSubscriber xSub;
  DoubleSubscriber zSub;
  DoubleSubscriber rotSub;
  DoubleArraySubscriber absPoseSub;

  DoubleSubscriber frameX;
  DoubleSubscriber frameZ;

  // IDK what this does, ask Caleb or Quin
  LinearFilter zFilter = LinearFilter.singlePoleIIR(0.5, 0.02);

  public Trajectory targetTrajectory;

  public VisionSubsystemLegacy() {

    // Do some fancy vision stuff, IDK (Looks like were receiving camera position data from NT)
    nt = NetworkTableInstance.getDefault();
    table = nt.getTable("SmartDashboard");

    xSub = table.getDoubleTopic("x").subscribe(0.0);
    zSub = table.getDoubleTopic("z").subscribe(0.0);
    rotSub = table.getDoubleTopic("rot").subscribe(0.0);
    absPoseSub = table.getDoubleArrayTopic("robotPose").subscribe(new double[0]);

    frameX = table.getDoubleTopic("frameX").subscribe(0);
    frameZ = table.getDoubleTopic("frameZ").subscribe(0);

    nt.startClient4("robot");
    nt.setServer("localhost"); // where TEAM=190, 294, etc, or use inst.setServer("hostname") or similar

  }

    public double getAprilTagX(int tagId) {
    //throw new Exception("Add logic for this method to return the x position of the AprilTag *in the camera frame*!");
    return frameX.get();
  }

  public double getAprilTagZ(int tagId) {
    return zFilter.calculate(frameZ.get());
  }


  /**
   * A method that gets the robot's absolute pose from visible AprilTags. 
   * 
   * @return the absolute pose of the robot based on visible AprilTags
   */
  public Pose2d getRobotPose() {

    //Index 0 is x, 1 is z, and 2 is the rotation in radians
    double[] poseArray = absPoseSub.get();

    if (poseArray.length <= 0) {
      return null;
    }

    Rotation2d rot = Rotation2d.fromRadians(poseArray[2]);
    return new Pose2d(poseArray[1], poseArray[0], rot);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
