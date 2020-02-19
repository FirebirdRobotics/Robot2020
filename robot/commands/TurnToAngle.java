/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

public class TurnToAngle extends CommandBase {

  private final Drivetrain m_drive;
  private double m_rotationSpeed;
  private double m_currentAngle;
  private final double m_targetAngle;
  private final AHRS m_gyro;

  private double m_error, m_integralError, m_derivativeError, m_previousError;
  private double kP, kI, kD;

  private boolean isFinished;

  // Angle needs to be in radians, as that is the superior unit mathematically
  // speaking. (lol)
  // RotateSpeed, on the other hand, is in ratio form (as used on all
  // SpeedControllers)
  public TurnToAngle(final Drivetrain dt, final AHRS gyro, final double rotationSpeed, final double targetAngle) {
    m_drive = dt;
    m_rotationSpeed = rotationSpeed;
    m_gyro = gyro;
    m_targetAngle = targetAngle;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println("target angle: " + m_targetAngle);
    // System.out.println("current angle: " + m_gyro.getAngle());

    // if angle is negative, make positive; then round to nearest whole angle
    m_currentAngle = Math.abs(Math.round(m_gyro.getAngle()));

    // CALCULATE ERROR
    m_error = m_targetAngle - m_currentAngle; // if error is positive, then OK; if error negative, we needa do some changes
    m_integralError += m_error * DriveConstants.kTimePerLoop;
    m_derivativeError = (m_previousError - m_error) / DriveConstants.kTimePerLoop;
    m_previousError = m_error;

    // CALCULATE SPEED USING PID
    m_rotationSpeed = (m_error * DriveConstants.kP) + (m_integralError * DriveConstants.kI) + (m_derivativeError * DriveConstants.kD); // PID
    m_rotationSpeed /= 35; // divide by this factor to convert angles into a usable speed ratio (no actual math involved, just a constant)
    m_rotationSpeed += DriveConstants.kMinimumSpeed; // minimum speed for robot to turn at

    if (m_currentAngle < m_targetAngle) { // if error is positive value


      // drive using calculated PID
      m_drive.autoDrive(0, m_rotationSpeed);
      isFinished = false;
    } else if (m_currentAngle > m_targetAngle) { // if error is negative value

      // drive using calculated PID
      m_drive.autoDrive(0, m_rotationSpeed);
      isFinished = false;
    } else {
      m_drive.autoDrive(0, 0);
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // reset all variables after command is done
    m_error = 0;
    m_integralError = 0;
    m_derivativeError = 0;
    m_previousError = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
