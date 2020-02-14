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

public class AutoShooterCommand extends HopperShooterCommand {

  public AutoShooterCommand(ShooterSystem shooter, HopperSystem hopper, VisionSystem vision, int numOfBalls) {
    super(shooter, hopper, numOfBalls, vision.rawDistanceToTarget());
    addRequirements(vision);
  }
}