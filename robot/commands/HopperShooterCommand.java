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

/**
 * Runs the shooter along with the hopper, so that the hopper pushes balls into the shooter at certain intervals.
 */

public class HopperShooterCommand extends ShooterCommand {

  private final HopperSystem m_hopper;
  private final Timer m_shooterTimer = new Timer();
  private final Timer m_hopperTimer = new Timer();
  private boolean shooterWarmupDone = false;
  private final int m_numOfBalls;
  private int m_count = 0;

  public HopperShooterCommand(ShooterSystem shooter, HopperSystem hopper, int numOfBalls, double distance) {
    // Calls the constructor
    super(shooter, distance);
    m_hopper = hopper;
    addRequirements(m_hopper);

    m_numOfBalls = numOfBalls;
    m_shooterTimer.start();
  }

  @Override
  public void initialize() {
    super.initialize();
    m_hopper.toggleSolenoid();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();
    if (m_shooterTimer.get() > AutonomousConstants.kShooterWarmUpTime) { 
      if (!shooterWarmupDone) {
        m_hopperTimer.start();
        shooterWarmupDone = true;
      }

      if (m_hopperTimer.get() < AutonomousConstants.kHopperWaitTime) { // wait for shooter to warm up
        m_hopper.setHopper(HopperConstants.kHopperSpeed); // run hopper for kHopperWaitTime
      } else if (m_hopperTimer.get() > AutonomousConstants.kShooterWaitTime) {
        m_count++;
        m_hopperTimer.reset(); // reset hopper timer if no ball is in shooter
        shooterWarmupDone = false;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterTimer.reset();
    m_hopperTimer.reset();
    m_hopper.toggleSolenoid();
    super.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_count >= m_numOfBalls);
  }
}