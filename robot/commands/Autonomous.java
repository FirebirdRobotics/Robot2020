/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.HopperSystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.ShooterSystem;
import frc.robot.subsystems.VisionSystem;

public class Autonomous extends SequentialCommandGroup {

  public Autonomous(Drivetrain drive, IntakeSystem intake, ShooterSystem shooter, VisionSystem vision, AHRS gyro, HopperSystem hopper, Trajectory... path) {

    // this makes it so that we can use as many trajectories as we want
    Trajectory[] m_trajectories = path;

    if (m_trajectories.length < 1) {
      throw new IllegalArgumentException("No trajectories found. Please include trajectories when creating an Autonomous");
    }

    addCommands(
      // 1ST PHASE OF AUTO - move to enemy trench & steal 2 balls
      new ParallelCommandGroup(
        // run path towards enemy trench
        RunPath.getPath(m_trajectories[0], drive).andThen(() -> drive.tankDriveVolts(0, 0), drive),
        // run intake, take in two balls
        new IntakeCommand(intake, hopper, 3, 2)
      ),

      // 2ND PHASE OF AUTO - move to shooting position & shoot
      RunPath.getPath(m_trajectories[1], drive).andThen(() -> drive.tankDriveVolts(0, 0), drive),
      // center with vision & shoot all balls
      new TurnToTarget(vision, drive),
      new AutoShooterCommand(shooter, hopper, vision, 5),

      // 3RD PHASE OF AUTO - move to pickup 5 more balls
      new ParallelCommandGroup(
        // run path to other balls
        RunPath.getPath(m_trajectories[2], drive).andThen(() -> drive.tankDriveVolts(0, 0), drive),
        // run intake, get 5 balls
        new IntakeCommand(intake, hopper, 3, 2)
      ),

      // 4TH PHASE OF AUTO - move to shooting position & shoot
      RunPath.getPath(m_trajectories[3], drive).andThen(() -> drive.tankDriveVolts(0, 0), drive),
      // center with vision & shoot all balls
      new TurnToTarget(vision, drive),
      new AutoShooterCommand(shooter, hopper, vision, 5)
    );
  }
}
