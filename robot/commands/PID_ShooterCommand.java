/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.PhysicsConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.UnitConversionConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.ShooterSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class PID_ShooterCommand extends PIDCommand {
  /**
   * Creates a new PID_ShooterCommand.
   */
  private ShooterSystem m_shooter;
  private final static double EPSILON = 0.01; // all error unaccounted for
  private double m_velocity;
  private final ShuffleboardTab m_teleopTab = Shuffleboard.getTab("Teleop");


  public PID_ShooterCommand(ShooterSystem shooter, double distance) {
    super(
        // The controller that the command will use
        new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD),
        // This should return the measurement
        () -> shooter.getSpeed(),
        // This should return the setpoint (can also be a constant)
        () -> calculateRequiredVelocity(distance),
        // This uses the output
        output -> {
          shooter.manualSpinMotor(output);
        });
    addRequirements(shooter);
    m_shooter = shooter;
    m_velocity = calculateRequiredVelocity(distance);
    // Configure additional PID options by calling `getController` here.
  }

  /**
   * Calculate intial velocity the projectile needs to have in order to reach the desired angle (get aquainted with physics).
   * We are assuming a fixed angle, which is the reason the equation only has one solution.
   * v = (distance * sqrt(g/2)) / (cos(angle) * sqrt(tan(angle)*distance + (height of shooter - height of target))) 
   */
  private static double calculateRequiredVelocity(double distanceToTarget) {
    double o = ShooterConstants.kShooterAngle * UnitConversionConstants.angleConversionFactor;
    double g = PhysicsConstants.gAcceleration;
    double d = distanceToTarget + FieldConstants.kOuterToInnerTarget;
    double hI = ShooterConstants.kShooterHeight;
    double hD = VisionConstants.kTargetHeight;
    double velocity = ((d * Math.sqrt(g * 0.5)) / (Math.cos(o) * Math.sqrt(Math.tan(o)*d + (hI - hD)))) + EPSILON;
    velocity = (velocity * 60) / (MotorConstants.kNeoRPM * ShooterConstants.kGearRatio * 2.0 * Math.PI * ShooterConstants.kMotorRadius);
    return velocity;
  }

  @Override
  public void execute() {
    super.execute();
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
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    m_shooter.manualSpinMotor(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}