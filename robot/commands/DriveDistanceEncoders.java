/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.Drivetrain;

public class DriveDistanceEncoders extends CommandBase {

  private final Drivetrain m_drivetrain;
  private double m_rotations;
  private double m_targetEncoderCounts;

  public DriveDistanceEncoders(Drivetrain dt, double targetDistance) {
    m_drivetrain = dt;
    addRequirements(m_drivetrain);

    // change target distance into number of rotations needed (distance / circumference of wheel)
    m_rotations = targetDistance / (2 * Math.PI * DriveConstants.kWheelRadius);

    // multiply by gear ratio (8.4 rotations of motor per 1 rotation of wheel)
    m_rotations *= DriveConstants.kGearRatio;

    // multiply rotations by encoder counts per rotation
    m_targetEncoderCounts = m_rotations * MotorConstants.kFalconCPR;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.driveWithEncoders(m_targetEncoderCounts);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_drivetrain.doneDrivingEncoder(m_targetEncoderCounts);
  }
}
