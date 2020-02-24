/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class TurnAngle extends CommandBase {

  private final Drivetrain m_drive;
  private final double m_rotationSpeed;
  private double m_targetAngle;
  private double m_initialAngle;
  private final AHRS m_gyro;

  private double targetZoneLower, targetZoneUpper;

  private boolean isFinished;

  // Angle needs to be in radians, as that is the superior unit mathematically speaking. (lol)
  // RotateSpeed, on the other hand, is in ratio form (as used on all SpeedControllers)
  public TurnAngle(Drivetrain dt, AHRS gyro, double rotationSpeed, double targetAngle) {
    m_drive = dt;
    m_rotationSpeed = rotationSpeed;
    m_gyro = gyro;
    m_targetAngle = targetAngle;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_initialAngle = m_gyro.getAngle();
    m_targetAngle += m_initialAngle;

    targetZoneLower = m_targetAngle - (0.5);
    targetZoneUpper = m_targetAngle + (0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println("target angle: " + m_targetAngle);
    // System.out.println("current angle: " + m_gyro.getAngle());

    if (m_gyro.getAngle() < targetZoneLower) {
      m_drive.autoDrive(0, m_rotationSpeed);
      isFinished = false;
    } else if (m_gyro.getAngle() > targetZoneUpper) {
      m_drive.autoDrive(0, -m_rotationSpeed);
      isFinished = false;
    } else {
      m_drive.autoDrive(0, 0);
      isFinished = true;
    }  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
