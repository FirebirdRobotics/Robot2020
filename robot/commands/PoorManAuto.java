/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.HopperSystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.ShooterSystem;
import frc.robot.subsystems.VisionSystem;

/**
 * Use this beautiful command to shocase the robots autonomous capabilities.
 * As the title suggests, this code is janky and rough around the edges, so beware.
 * In the future, we will be using PathWeaver, which has not been tested yet.
 */

public class PoorManAuto extends SequentialCommandGroup {

  public PoorManAuto(Drivetrain drive, IntakeSystem intake, ShooterSystem shooter, VisionSystem vision, AHRS gyro, HopperSystem hopper) {

    addCommands(
      // 1ST PHASE OF AUTO - move to enemy trench & steal 2 balls
      new DriveDistanceEncoders(drive, 24),
      new TurnToTarget(vision, drive),
      new AutoShooterCommand(shooter, hopper, vision, 5),
      new TurnAngle(drive, gyro, 0.2, 180)
    );
  }
}
