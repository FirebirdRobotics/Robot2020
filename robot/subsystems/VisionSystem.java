/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSystem extends SubsystemBase {

  /*
   * NetworkTables is a collection of databases stored in the RoboRIO The
   * coprocessor can change data inside of the rio This database can be viewed in
   * a program called OutlineViewer
   */
  private final NetworkTable m_limelight;
  private boolean m_limelightHasValidTarget;
  private double tv, tx, ty, ta;
  private double error, integralError, derivativeError, previousError;

  private double m_steerAdjust; // amount to rotate drivetrain
  private double m_driveAdjust; // amount to drive drivetrain

  public VisionSystem() {
    // Points "m_limelight" to the limelight database
    m_limelight = NetworkTableInstance.getDefault().getTable("limelight");

    // Configure limelight-
    // http://docs.limelightvision.io/en/latest/networktables_api.html
    m_limelight.getEntry("camMode").setNumber(0);
    m_limelight.getEntry("stream").setNumber(1);

    m_limelightHasValidTarget = false;
  }

  public void visionRoutineTape(Drivetrain drivetrain) {
    updateLimelightTracking();

    // Start with proportional steering
    double m_rotationError = tx * VisionConstants.kpRotation + VisionConstants.kConstantForce;
    m_steerAdjust = m_rotationError;

    // calculate PID stuff
    error = VisionConstants.closestTargetArea - ta; // error = target - actual
    integralError += error * VisionConstants.kTimePerLoop; // integral increased by error * time (0.02 seconds per loop)
    derivativeError = (error - previousError) / .02; // derivative = change in error / time (0.02 seconds per loop)

    // try to drive forward until the target area reaches our desired area
    double m_distanceError = (error * VisionConstants.kpDistance) + (integralError * VisionConstants.kiDistance)
        + (derivativeError * VisionConstants.kdDistance) + VisionConstants.kConstantForce;

    // don't let the robot drive too fast into the goal
    if (m_distanceError > VisionConstants.kMaxDrive) {
      m_distanceError = VisionConstants.kMaxDrive;
    } else if (m_distanceError < -VisionConstants.kMaxDrive) {
      m_distanceError = -VisionConstants.kMaxDrive;
    }

    m_driveAdjust = m_distanceError;

    previousError = error; // update previousError to current error

    if (m_limelightHasValidTarget) { // if limelight sees target
      drivetrain.arcadeDrive(m_driveAdjust, m_steerAdjust); // drive using command-tuned values
    } else {
      drivetrain.arcadeDrive(0.0, 0.0); // otherwise do nothing
    }
  }

  public void visionRoutineReleased(Drivetrain m_drive) {
    m_driveAdjust = 0.0;
    m_steerAdjust = 0.0;
    m_drive.arcadeDrive(0.0, 0.0);
  }

  public void updateVisionMeasurements() {
    tv = m_limelight.getEntry("tv").getDouble(0);
    tx = m_limelight.getEntry("tx").getDouble(0);
    ty = m_limelight.getEntry("ty").getDouble(0);
    ta = m_limelight.getEntry("ta").getDouble(0);
  }

  public void updateLimelightTracking() {
    updateVisionMeasurements();

    SmartDashboard.putNumber("Target Detected", tv);
    SmartDashboard.putNumber("Horizontal Error", tx);
    SmartDashboard.putNumber("Vertical Error", ty);
    SmartDashboard.putNumber("Target Area", ta);
    SmartDashboard.putNumber("Closest Target (Area)", VisionConstants.closestTargetArea);
    SmartDashboard.putNumber("Closest Target (Distance)", VisionConstants.closestTargetDistance);

    // using target area
    VisionConstants.closestTargetArea = getClosestTargetArea(VisionConstants.kTargetAreas, ta);

    // using target distance
    VisionConstants.closestTargetDistance = getClosestTargetDistance(VisionConstants.kTargetDistances,
        distanceToTarget(ty));

    if (tv < 1.0) { // IF TARGET IS NOT DETECTED
      m_limelightHasValidTarget = false;
      m_driveAdjust = 0.0;
      m_steerAdjust = 0.0;
      return;
    }

    m_limelightHasValidTarget = true;
  }

  // function that returns the value in kTargetAreas closest to your robot
  public double getClosestTargetArea(double[] targetAreas, double currentTargetArea) {
    // setup the array containing distances to targets
    for (int i = 0; i < VisionConstants.kTargetAreas.length; i++) {
      // if currentTA above target (array value), will give back negative value
      // if currentTA below target (array value), will give back positive value
      double thisTargetArea = VisionConstants.kTargetAreas[i] - currentTargetArea;
      VisionConstants.distancesToTargets[i] = thisTargetArea; // set index i of dist targets array
    }

    // find lowest distance to targets
    double lowestDistance = 100.0; // arbitrary initial value
    for (int i = 0; i < VisionConstants.kTargetAreas.length; i++) {
      // take abs value of distance to target (read above comments)
      if (Math.abs(VisionConstants.distancesToTargets[i]) < lowestDistance) {
        lowestDistance = VisionConstants.distancesToTargets[i];
      }
    }

    // now with lowest distance get index of target area
    int targetIndex = 0;
    for (int i = 0; i < VisionConstants.kTargetAreas.length; i++) {
      if (lowestDistance == VisionConstants.distancesToTargets[i]) {
        targetIndex = i;
      }
    }

    return VisionConstants.kTargetAreas[targetIndex];
  }

  // same function as above but uses target distance instead of the targetArea
  // calculated by limelight
  public double getClosestTargetDistance(double[] targetAreas, double currentDistance) {
    // setup the array containing distances to targets
    for (int i = 0; i < VisionConstants.kTargetDistances.length; i++) {
      // if currentDist above target (array value), will give back negative value
      // if currentDist below target (array value), will give back positive value
      double thisTargetDistance = VisionConstants.kTargetDistances[i] - currentDistance;
      VisionConstants.distancesToTargets[i] = thisTargetDistance; // set index i of dist to targets array
    }

    // find lowest distance to targets
    double lowestDistance = 100.0; // arbitrary initial value
    for (int i = 0; i < VisionConstants.kTargetDistances.length; i++) {
      // take abs value of distance to target (read above comments)
      if (Math.abs(VisionConstants.distancesToTargets[i]) < lowestDistance) {
        lowestDistance = VisionConstants.distancesToTargets[i];
      }
    }

    // now with lowest distance get index of target distance
    int targetIndex = 0;
    for (int i = 0; i < VisionConstants.kTargetDistances.length; i++) {
      if (lowestDistance == VisionConstants.distancesToTargets[i]) {
        targetIndex = i;
      }
    }

    return VisionConstants.kTargetDistances[targetIndex];
  }

  public double distanceToTarget(double targetAngle) {
    /*
     * Math: http://docs.limelightvision.io/en/latest/cs_estimating_distance.html
     * 
     * tangent = opposite / adjacent tan(a1+a2) = (h2-h1) / d
     * 
     * where: a1 = angle the camera is mounted at (probably somewhere between
     * 0-90deg) a2 = angle of the camera to the target (its the "ty" value output by
     * limelight) h2 = height of the target h1 = height of the camera
     * 
     * distance of camera to target = height from camera to target / angle of camera
     * to target d = (h2-h1) / tan(a1+a2)
     */

    // the only thing that changes in this equation is targetAngle, which is ty in
    // this project
    return (VisionConstants.kTargetHeight - VisionConstants.kCameraHeight)
        / Math.tan(VisionConstants.kMountingAngle + targetAngle);
  }

  public double rawDistanceToTarget() {
    updateVisionMeasurements();
    return distanceToTarget(ty);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}