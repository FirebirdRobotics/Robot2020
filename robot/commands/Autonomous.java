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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.HopperSystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.ShooterSystem;
import frc.robot.subsystems.VisionSystem;

public class Autonomous extends SequentialCommandGroup {

  public Autonomous(Drivetrain drivetrain, IntakeSystem intake, ShooterSystem shooter, VisionSystem vision, AHRS gyro, HopperSystem hopper, Trajectory... path) {

    // this makes it so that we can use as many trajectories as we want
    Trajectory[] m_trajectories = path;

    if (m_trajectories.length < 1) {
      throw new IllegalArgumentException("No trajectories found. Please include trajectories when creating an Autonomous");
    }

    addCommands(
      // first phase of DT motion - move to enemy trench
      RunPath.getPath(m_trajectories[0], drivetrain),
      
      // run intake

      // second phase of DT motion - move to shooting position
      RunPath.getPath(m_trajectories[1], drivetrain),

      // center with vision & shoot all balls
      new AutoShooterCommand(shooter, hopper, vision, 5),

      // third phase of DT motion - move to pickup more balls
      RunPath.getPath(m_trajectories[3], drivetrain),
      
      // run intake

      // fourth phase of DT motion - move to shooting position
      RunPath.getPath(m_trajectories[4], drivetrain),

      // center with vision & shoot all balls
      new AutoShooterCommand(shooter, hopper, vision, 5)
    );
  }
}
