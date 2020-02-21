/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSystem;
import frc.robot.subsystems.VisionSystem;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.PhysicsConstants;
import frc.robot.Constants.VisionConstants;

public class ShooterCommand extends CommandBase {

  private ShooterSystem m_shooter;
  private VisionSystem m_vision;
  private double distanceToTarget;
  private boolean triggerFinished = false;
  private final double EPSILON = 0.01; // all error unaccounted for

  /**
   * Creates a new ShooterCommand.
   */
  public ShooterCommand(ShooterSystem shooter, double distance) {
    m_shooter = shooter;
    distanceToTarget = distance;
    addRequirements(m_shooter, m_vision);
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
     */
    double velocity = (distanceToTarget / Math.cos(ShooterConstants.kShooterAngle))
        * Math.sqrt(PhysicsConstants.gAcceleration / (2 * (Math.tan(ShooterConstants.kShooterAngle) * distanceToTarget
            + (ShooterConstants.kShooterHeight - VisionConstants.kTargetHeight)))) + EPSILON;

    // convert velocity (in m/s) into unitless ratio
    velocity /= (MotorConstants.kNeoRPM * ShooterConstants.kGearRatio * 2.0 * Math.PI / (60)) * ShooterConstants.kMotorRadius;

    // don't run motor greater than max speed (on -1 to 1 ratio)
    if (velocity > 1)
      velocity = 1;
    if (velocity < -1)
      velocity = -1;

    // spin motor at calculated velocity
    m_shooter.manualSpinMotor(velocity);

    // sets target goal for PID Controller
    m_shooter.setSetpoint(velocity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Starts PID loop.
    m_shooter.enable();
    SmartDashboard.putNumber("Current Speed", m_shooter.getSpeed()*MotorConstants.kNeoRPM);
    SmartDashboard.putNumber("Shooter spin ratio", m_shooter.getSpeed() / m_velocity);
    // SmartDashboard.putNumber("Shooter Test", 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Ends PID loop.
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
