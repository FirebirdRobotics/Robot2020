/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIREBIRD. All Rights Reserved. credit to FIRST*/
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.HopperSystem;
import frc.robot.subsystems.ShooterSystem;
import frc.robot.subsystems.VisionSystem;
import frc.robot.subsystems.Drivetrain;

/**
 * Extends HopperShooterCommand and takes in vision system in order to calculate the distance to the target.
 * Since it is an extension of ^ , it also spins the hopper as necessary.
 */

public class AutoShooterCommand extends HopperShooterCommand {

  private VisionSystem m_vision;
  private Drivetrain m_drive;

  public AutoShooterCommand(ShooterSystem shooter, HopperSystem hopper, VisionSystem vision, int numOfBalls) {
    super(shooter, hopper, numOfBalls, vision.rawDistanceToTarget());
    m_vision = vision;
    addRequirements(m_vision);
  }

  @Override 
  public void execute() {
    m_vision.turnToTarget(m_drive);
    super.execute();
  }
}
