/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

/** 
 * This class (and its one method) returns a ramsete controller that can simply run a sent in trajectory using the drivetrain;
 * Trajectories are generated by PathWeaver and placed in the deploy folder of the project.
 * 
 * Read more about Ramsete Commands and Controllers here: https://docs.wpilib.org/en/latest/docs/software/advanced-control/trajectories/ramsete.html
 * 
 * I got this code from here: https://github.com/NAHSRobotics-Team5667/FRC_2020/tree/master/src/main/java/frc/robot/autos
 * 
 * Essentially, this replaces the hard-coding of paths that we would do using something like the DriveDistanceEncoders command.
 * 
 * The main advantages to this are:
 *    - Faster autonomous overall
 *    - Easier to get working
 *    - Allows for more complexity overall
 */
public class RunPath {
  public static RamseteCommand getPath(Trajectory path, Drivetrain drive) {
    drive.resetOdometry(path.getInitialPose());

    return new RamseteCommand(
      path, 
      drive::getPose, 
      new RamseteController(AutonomousConstants.kRamseteB, AutonomousConstants.kRamseteZeta), 
      new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
          DriveConstants.kaVoltSecondsSquaredPerMeter), 
      DriveConstants.kDriveKinematics, 
      drive::getWheelSpeeds, 
      new PIDController(DriveConstants.kPDriveVel, 0, 0), 
      new PIDController(DriveConstants.kPDriveVel, 0, 0), 
      drive::tankDriveVolts, drive);
  }
}
