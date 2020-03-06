/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ColorWheelConstants;
import frc.robot.subsystems.ColorWheelSystem;

public class SpinRotations extends CommandBase {
  
  private ColorWheelSystem m_color;
  private double target;
  private double m_rotations;
  
  public SpinRotations(ColorWheelSystem color, double rotations) {
    m_color = color;
    addRequirements(m_color);
    target = rotations * ColorWheelConstants.kCountsPerRotation;
    m_rotations = rotations;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_color.getEncoder().setPosition(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_color.setColorSpinner(ColorWheelConstants.kColorSpinnerSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_color.setColorSpinner(0.0);
    m_color.getEncoder().setPosition(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return target >= (ColorWheelConstants.kCountsPerRotation * m_rotations);
  }
}
