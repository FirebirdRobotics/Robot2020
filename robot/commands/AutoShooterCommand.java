/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.HopperConstants;
import frc.robot.subsystems.HopperSystem;
import frc.robot.subsystems.ShooterSystem;
import frc.robot.subsystems.VisionSystem;

public class AutoShooterCommand extends ShooterCommand {
  /**
   * Creates a new AutoShooterCommand.
   */

  private final HopperSystem m_hopper;
  private final Timer m_shooterTimer = new Timer();
  private final Timer m_hopperTimer = new Timer();

  public AutoShooterCommand(ShooterSystem shooter, HopperSystem hopper, VisionSystem vision) {
    super(shooter, vision);
    m_hopper = hopper;
    addRequirements(m_hopper);

    m_shooterTimer.start();
    m_hopperTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();
    if (m_shooterTimer.get() > AutonomousConstants.kShooterWarmUpTime) { // wait for shooter to warm up
      if (m_hopperTimer.get() < AutonomousConstants.kHopperWaitTime) {
        m_hopper.runHopper(HopperConstants.kHopperSpeed); // run hopper for kHopperWaitTime
      } else if (m_hopperTimer.get() > AutonomousConstants.kShooterWaitTime) {
        m_hopperTimer.reset(); // reset hopper timer if no ball is in shooter
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterTimer.reset();
    m_hopperTimer.reset();
    super.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_hopper.empty();
  }
}