/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIREBIRD. All Rights Reserved. credit to FIRST*/
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.HopperSystem;
import frc.robot.subsystems.IntakeSystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.HopperConstants;

/**
 * Runs the intake after a specified period of time.
 */
public class IntakeCommand extends CommandBase {

  private IntakeSystem m_intake;
  private HopperSystem m_hopper;
  private Timer m_timer = new Timer();
  private double m_waitTime;
  private double m_runTime;

  /**
   * Runs the intake with the hopper after waitTime for a period of runTime
   * @param intake The intake subsystem to use.
   * @param hopper The hopper subsystem to use.
   * @param waitTime The time the command waits before running the motors.
   * @param runTime The time the intake motors will run.
   */
  public IntakeCommand(IntakeSystem intake, HopperSystem hopper, double waitTime, double runTime) {
    m_intake = intake;
    m_hopper = hopper;
    m_waitTime = waitTime;
    m_runTime = runTime;
  }

  @Override
  public void initialize() {
    m_timer.start();
    m_hopper.setSolenoid(true); // true is down (closed)
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override 
  public void execute() {
    if (m_timer.get() <= m_waitTime) {;}
    else {
        m_intake.setIntake(IntakeConstants.kIntakeSpeed);
        m_hopper.setHopper(HopperConstants.kHopperSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_hopper.setHopper(0);
    m_intake.setIntake(0);
    m_timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_timer.get() >= m_runTime);
  }
}