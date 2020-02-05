/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ShooterSystem;
import frc.robot.subsystems.VisionSystem;

public class Autonomous extends SequentialCommandGroup {

  public Autonomous(Drivetrain drivetrain, ShooterSystem shooter, VisionSystem vision, AHRS gyro) {
    addCommands(
      new DriveDistance(12, AutonomousConstants.kDriveSpeed, drivetrain), // distance in inches
      new TurnToAngle(drivetrain, gyro, AutonomousConstants.kTurnSpeed, 180),
      new InstantCommand(() -> vision.visionRoutineTape(drivetrain), drivetrain),
      new ShooterCommand(shooter, vision)

    );
  }
}
