/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSystem;

public class LiftElevator extends CommandBase {
  
  private final ClimbSystem m_climb;
  private boolean m_stop;
  private double m_position;

  public LiftElevator(ClimbSystem climb, double position) {
    m_climb = climb;
    m_position = position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_stop = !m_climb.setElevatorPosition(m_position);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climb.releasedElevator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_stop;
  }
}
