/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MotorConstants;
import edu.wpi.first.wpilibj.Timer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html

/**
 * Uses both a PID loop and rotational physics to force the robot to drive a certain distance. 
 * This DOES NOT use Encoders, and is therefore less accurate than DriveDistanceEncoders;
 * however, use this in case Encoders break down.
 */

public class NiceDriveDistance extends PIDCommand {
  /**
   * Creates a new NiceDriveDistance.
   */

   private Drivetrain m_drive;
   private Timer m_timer = new Timer();
   private double initialDistance;
   private double midDistance;
   private double totalDistance;
   private double d_distance;
   private boolean isTrigger = true;
   private double m_time;
   private double m_velocity;

  /**
    * 
    * @param dt The drivetrain to be used for driving.
    * @param velocity The average velocity at which one desires to drive.
    * @param distance The total distance to be driven.
    */
  public NiceDriveDistance(Drivetrain dt, double velocity, double distance) {
    super(
        // The controller that the command will use
        new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD),
        // This should return the measurement
        () -> {return dt.getVelocity();},
        // This should return the setpoint (can also be a constant)
        velocity,
        // This uses the output
        (output) -> {dt.arcadeDrive(output,0);}, 
        dt);
    m_drive = dt;
    m_velocity = velocity;
    d_distance = distance;
    initialDistance = m_drive.getRightEncoderPosition2() * DriveConstants.kWheelRadius * DriveConstants.kGearRatio / MotorConstants.kFalconCPR;
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  @Override
  public void execute() {
    super.execute();
    if (this.getController().atSetpoint() && isTrigger) {
        midDistance = (m_drive.getRightEncoderPosition2() * DriveConstants.kWheelRadius * DriveConstants.kGearRatio / MotorConstants.kFalconCPR);
        totalDistance = d_distance - (midDistance - initialDistance);
        m_time = ((totalDistance * DriveConstants.kGearRatio * 60) / ((m_velocity * MotorConstants.kFalconRPM) * ((2 * Math.PI)) * DriveConstants.kWheelRadius));
        m_timer.start();
        isTrigger = false;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_timer.get() > m_time);
  }
}
