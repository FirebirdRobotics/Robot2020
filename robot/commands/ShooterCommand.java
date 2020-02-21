/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSystem;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.PhysicsConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.UnitConversionConstants;
import frc.robot.Constants.FieldConstants;


public class ShooterCommand extends CommandBase {

  private ShooterSystem m_shooter;
  private double distanceToTarget;
  private final double EPSILON = 0.01; // all error unaccounted for
  private double m_velocity;
  private boolean triggerFinished = false;

  /**
   * Creates a new ShooterCommand.
   */
  public ShooterCommand(ShooterSystem shooter, double distance) {
    m_shooter = shooter;
    distanceToTarget = distance;
    addRequirements(m_shooter);
    m_velocity = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // get distance to target
    //distanceToTarget = m_vision.rawDistanceToTarget();

    /*
     * get velocity required to get to target:
     * 
     * velocity = distance / (time * cos(angle of shooter))
     * 
     * time = sqrt(2 * (tangent(angle of shooter) * distance + height of shooter - height of target)) / gravity
     * 
     * velocity = distance / (whatever ^ this is * cos(angle of shooter))
     * 
     * Or better yet: v = (distance * sqrt(g/2)) / (cos(angle) * sqrt(tan(angle)*distance + (height of shooter - height of target)))
     * 
     */

    {
      double o = ShooterConstants.kShooterAngle * UnitConversionConstants.angleConversionFactor;
      double g = PhysicsConstants.gAcceleration;
      double d = distanceToTarget + FieldConstants.kOuterToInnerTarget;
      double hI = ShooterConstants.kShooterHeight;
      double hD = VisionConstants.kTargetHeight;

      m_velocity = ((d * Math.sqrt(g * 0.5)) / (Math.cos(o) * Math.sqrt(Math.tan(o)*d + (hI - hD)))) + EPSILON;
    }

    // double velocity = (distanceToTarget / Math.cos(ShooterConstants.kShooterAngle * UnitConversionConstants.angleConversionFactor))
    //     * Math.sqrt(PhysicsConstants.gAcceleration / (2 * (Math.tan(ShooterConstants.kShooterAngle * UnitConversionConstants.angleConversionFactor) * distanceToTarget
    //         + (ShooterConstants.kShooterHeight - VisionConstants.kTargetHeight)))) + EPSILON;

    // convert velocity (in m/s) into unitless ratio
    m_velocity = (m_velocity * 60) / (MotorConstants.kNeoRPM * ShooterConstants.kGearRatio * 2.0 * Math.PI * ShooterConstants.kMotorRadius);

    SmartDashboard.putNumber("Calculated Shooter SetPoint", m_velocity);

    // don't run motor greater than max speed (on -1 to 1 ratio)
    // can be changed to maximum safe velocity
    if (m_velocity > 1)
      m_velocity = 1;
    if (m_velocity < -1)
      m_velocity = -1;

    // spin motor at calculated velocity
    m_shooter.manualSpinMotor(m_velocity);

    // sets target goal for PID Controller
    m_shooter.setSetpoint(m_velocity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Current Shooter Speed", m_shooter.getSpeed()*MotorConstants.kNeoRPM);
    SmartDashboard.putNumber("Shooter spin ratio", m_shooter.getSpeed() / m_velocity);
    // Starts PID loop.
    m_shooter.enable();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Ends PID loop.
    m_velocity = 0;
    m_shooter.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return triggerFinished;
  }

  public void triggerFinished () {
    triggerFinished = true;
  }
}
