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

public class TurnToAngle extends CommandBase {

  private final Drivetrain m_drive;
  private final double m_rotationSpeed;
  private final double m_targetAngle;
  private final double m_initialAngle;
  private final AHRS m_gyro;
  
  // Forget perfection. For convinience, just use degrees. I give up.
  /*
   * Assume that command does not take into account direction of the robot.
   * For example, inputting 180 degrees into the command will literally make the robot make half a turn.
  */
  // RotateSpeed, on the other hand, is in ratio form (as used on all SpeedControllers)
  public TurnToAngle(Drivetrain dt, AHRS gyro, double rotationSpeed, double targetAngle) {
    m_drive = dt;
    m_rotationSpeed = rotationSpeed;
    m_gyro = gyro;
    m_initialAngle = gyro.getAngle(); 
    m_targetAngle = m_initialAngle + targetAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.arcadeDrive(0, m_rotationSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_gyro.getAngle() >= m_targetAngle) return true;
    return false;
  }
}
