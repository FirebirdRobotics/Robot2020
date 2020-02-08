/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.HopperConstants;
import frc.robot.subsystems.HopperSystem;
import frc.robot.subsystems.ShooterSystem;
import frc.robot.subsystems.VisionSystem;

public class AutoShooterCommand extends ShooterCommand {
  /**
   * Creates a new AutoShooterCommand.
   */

  private final HopperSystem m_hopper;
  private final Timer m_timer = new Timer();
  private final Timer ms_timer = new Timer();
  private final double shooterWarmUpTime = 0.5;
  private final double hopperWaitTime = 2;
  private final double shooterWaitTime = 2;


  public AutoShooterCommand(ShooterSystem tShooter, HopperSystem tHopper, VisionSystem tVision) {
    super (tShooter, tVision);
    m_hopper = tHopper;
    m_timer.start();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();
    if (m_timer.get() < shooterWarmUpTime) {
      if (ms_timer.get() < hopperWaitTime) m_hopper.runHopper(HopperConstants.kHopperSpeed);
      else{
        if (ms_timer.get() < shooterWaitTime) {}
        else ms_timer.reset();
      } 
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.reset();
    super.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_hopper.full()) return true;
    return false;
  }
}