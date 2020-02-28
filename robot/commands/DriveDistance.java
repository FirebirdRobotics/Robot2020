/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.Drivetrain;

/**
 * Very inaccurate way to drive a certain distance. This command assumed that the motors would output perfect values without PID of any sort.
 * @deprecated Broken
 */

public class DriveDistance extends CommandBase {
  private Timer m_timer;
  private double gearRatio = 8.4;
  //private double epsilon;
  private Drivetrain m_drive;
  private double m_time;
  private double m_velocity = 0.2;
  private double m_distance = 24;

  public DriveDistance(Drivetrain dt) {
    m_timer = new Timer();
    
    //epsilon = -4.17 * m_velocity + 2.341;
    // if (velocity < -1) velocity = -1;
    // if (velocity > 1) velocity = 1;
    m_time = ((m_distance * gearRatio * 60) / ((m_velocity * MotorConstants.kFalconRPM) * ((2 * Math.PI)) * DriveConstants.kWheelRadius));
    
    //System.out.println("calculation: " + m_time);
    m_drive = dt;
    //m_velocity = velocity;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
    //System.out.println("initializing command");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.autoDrive(m_velocity, 0);
    System.out.println("target time: " + m_time);
    //System.out.println("time: " + m_timer.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_time < m_timer.get());
  }
}
