/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.Drivetrain;

public class DriveDistance extends CommandBase {
  
  private final Timer m_timer = new Timer();
  private final double m_time;
  private final Drivetrain m_drive;
  private final double m_velocity;
  
  // DISTANCE IN INCHES
  public DriveDistance(double distance, double velocity, Drivetrain dt) {
    if (velocity < -1) velocity = -1;
    if (velocity > 1) velocity = 1;

    /*
     * time = distance / velocity
     * 
     * velocity passed in as ratio (-1 to 1) so need to convert to RPM
     * 
     * (ratio velocity) * (max RPM of motor) * (2pi to convert to radians) * (1/60 to convert to minutes)
     */
    m_time = distance / (velocity * MotorConstants.kFalconRPM * ((2 * Math.PI) / 60) * AutonomousConstants.kWheelRadius);
    SmartDashboard.putNumber("Time of drive (auto)", m_time);
    m_drive = dt;
    m_velocity = velocity;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_timer.get() < m_time) {
      m_drive.arcadeDrive(m_velocity, 0);
    }
    System.out.println("time: " + m_timer.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_timer.get() > m_time);
  }
}
