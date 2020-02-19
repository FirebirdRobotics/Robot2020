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
import frc.robot.Constants.UnitConversionConstants;
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
  private double driveError, driveIntegralError, driveDerivativeError, drivePreviousError;
  private double turnError, turnIntegralError, turnDerivativeError, turnPreviousError;

  private double m_steerAdjust; // amount to rotate drivetrain
  private double m_driveAdjust; // amount to drive drivetrain

  public VisionSystem() {
    // Points "m_limelight" to the limelight database
    m_limelight = NetworkTableInstance.getDefault().getTable("limelight");

    // Configure limelight-
    // http://docs.limelightvision.io/en/latest/networktables_api.html
    m_limelight.getEntry("camMode").setNumber(0);
    m_limelight.getEntry("stream").setNumber(1);

    // set the pipeline to vision tracking OFF (will turn on when centering command is on)
    setPipeline(1);

    m_limelightHasValidTarget = false;
  }

  public void visionRoutineTape(Drivetrain drivetrain) {
    updateLimelightTracking();

    // calculate PID error for turning (probs only use P loop or PI loop)
    turnError = tx; // error = target - actual (but here it's calculated by limelight for us, so this is just a placeholder)
    turnIntegralError = turnError * VisionConstants.kTimePerLoop; // integral increased by error * time (0.02 seconds per loop)
    turnDerivativeError = (turnError - turnPreviousError) / VisionConstants.kTimePerLoop; // derivative = change in error / time (0.02 seconds per loop)
    turnPreviousError = turnError; // update previousError to current error

    // Start with proportional steering
    double m_rotationError = (turnError * VisionConstants.kpRotation) + (turnIntegralError * VisionConstants.kiRotation)
        + (turnDerivativeError * VisionConstants.kdRotation) + VisionConstants.kConstantForce;
    
    // limit turn speed to the max speed
    if (m_rotationError > VisionConstants.kMaxTurn) {
      m_rotationError = VisionConstants.kMaxTurn;
    } else if (m_rotationError < -VisionConstants.kMaxTurn) {
      m_rotationError = -VisionConstants.kMaxTurn;
    }
    m_steerAdjust = m_rotationError;

    // calculate PID error for forward/backward motion
    driveError = VisionConstants.closestTargetArea - ta; // error = target - actual
    driveIntegralError += driveError * VisionConstants.kTimePerLoop; // integral increased by error * time (0.02 seconds per loop)
    driveDerivativeError = (driveError - drivePreviousError) / VisionConstants.kTimePerLoop; // derivative = change in error / time (0.02 seconds per loop)
    drivePreviousError = driveError; // update previousError to current error

    // drive forward/backward until we reach our desired distance
    double m_distanceError = (driveError * VisionConstants.kpDistance) + (driveIntegralError * VisionConstants.kiDistance) 
        + (driveDerivativeError * VisionConstants.kdDistance) + VisionConstants.kConstantForce;

    // don't let the robot drive too fast into the goal
    if (m_distanceError > VisionConstants.kMaxDrive) {
      m_distanceError = VisionConstants.kMaxDrive;
    } else if (m_distanceError < -VisionConstants.kMaxDrive) {
      m_distanceError = -VisionConstants.kMaxDrive;
    }
    m_driveAdjust = m_distanceError;

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
    setPipeline(1);
  }

  public void turnToTarget(Drivetrain drivetrain) {
    updateLimelightTracking();

    double isNegative;
    if (tx < 0) {
      isNegative = -1;
    } else {
      isNegative = 1;
    }

    double txRatio = Math.abs(tx / 90);
    txRatio += 0.1;
    txRatio *= isNegative;

    double m_rotationError = txRatio;
    m_steerAdjust = m_rotationError;

    if (m_limelightHasValidTarget) { // if limelight sees target
      drivetrain.autoDrive(0, m_steerAdjust); // drive using command-tuned values
    } else {
      drivetrain.autoDrive(0.0, 0.0); // otherwise do nothing
    }
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

    // set pipeline to vision processing
    setPipeline(0);

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
     * where: a1 = angle the camera is mounted at (probably somewhere between 0-90deg) 
     *        a2 = angle of the camera to the target (its the "ty" value output by limelight) 
     *        h2 = height of the target h1 = height of the camera
     * 
     * distance of camera to target = height from camera to target / angle of camera to target 
     * d = (h2-h1) / tan(a1+a2)
     */

    // the only thing that changes in this equation is targetAngle, which is ty in this project
    return (VisionConstants.kTargetHeight - VisionConstants.kLimelightHeight)
        / Math.tan((VisionConstants.kMountingAngle + targetAngle) * UnitConversionConstants.angleConversionFactor);
  }

  public double rawDistanceToTarget() {
    updateVisionMeasurements();
    return distanceToTarget(ty);
  }

  public void setPipeline(int pipeline) {
    m_limelight.getEntry("pipeline").setNumber(pipeline);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}