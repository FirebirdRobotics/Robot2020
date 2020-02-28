/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DeprecatedShooterSystem;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.PhysicsConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.UnitConversionConstants;
import frc.robot.Constants.FieldConstants;


public class DeprecatedShooterCommand extends CommandBase {

  private DeprecatedShooterSystem m_shooter;
  private double distanceToTarget;
  private final double EPSILON = 0.01; // all error unaccounted for
  private double m_velocity;
  private boolean triggerFinished = false;
  private final ShuffleboardTab m_teleopTab = Shuffleboard.getTab("Teleop");

  /**
   * Creates a new ShooterCommand.
   * @param shooter The desired ShooterSystem to be spun.
   * @param distance The horizontal distance to the target.
   * @deprecated Preferably use PID_ShooterCommand instead, since this probably does not work.
   */
  public DeprecatedShooterCommand(DeprecatedShooterSystem shooter, double distance) {
    m_shooter = shooter;
    distanceToTarget = distance;
    addRequirements(m_shooter);
    m_velocity = 0;
  }

  /**
   * Calculate intial velocity the projectile needs to have in order to reach the desired angle (get aquainted with physics).
   * We are assuming a fixed angle, which is the reason the equation only has one solution.
   * v = (distance * sqrt(g/2)) / (cos(angle) * sqrt(tan(angle)*distance + (height of shooter - height of target))) 
   */

  @Override
  public void initialize() {
    {
      double o = ShooterConstants.kShooterAngle * UnitConversionConstants.angleConversionFactor;
      double g = PhysicsConstants.gAcceleration;
      double d = distanceToTarget + FieldConstants.kOuterToInnerTarget;
      double hI = ShooterConstants.kShooterHeight;
      double hD = VisionConstants.kTargetHeight;

      m_velocity = ((d * Math.sqrt(g * 0.5)) / (Math.cos(o) * Math.sqrt(Math.tan(o)*d + (hI - hD)))) + EPSILON;
    }

    // convert velocity (in m/s) into unitless ratio
    m_velocity = (m_velocity * 60) / (MotorConstants.kNeoRPM * ShooterConstants.kGearRatio * 2.0 * Math.PI * ShooterConstants.kMotorRadius);


    m_teleopTab.addNumber("Calculated Shooter Setpoint", new DoubleSupplier(){
      @Override
      public double getAsDouble() {
        return m_velocity;
      }
    });

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
    m_teleopTab.addNumber("Current Shooter Speed", new DoubleSupplier(){
      @Override
      public double getAsDouble() {
        return m_shooter.getSpeed() * MotorConstants.kNeoRPM;
      }
    });
    m_teleopTab.addNumber("Shooter Spin Efficiency", new DoubleSupplier(){
      @Override
      public double getAsDouble() {
        return m_shooter.getSpeed() *100 / m_velocity;
      }
    });
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
