/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    m_robotContainer.getGyro().reset();
    m_robotContainer.getVisionSystem().setPipeline(1);

    Shuffleboard.selectTab("Auto");
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    m_robotContainer.getVisionSystem().setPipeline(1);
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    m_robotContainer.getGyro().reset();
    m_robotContainer.getVisionSystem().setPipeline(1);

    Shuffleboard.selectTab("Auto");

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    m_robotContainer.getGyro().reset();
    m_robotContainer.getVisionSystem().setPipeline(1);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    /*
    SmartDashboard.putNumber("Shooter Speed", 0);
    SmartDashboard.putNumber("test", 0);
    SmartDashboard.putNumber("Drive kP", 0);
    SmartDashboard.putNumber("Drive kI", 0);
    SmartDashboard.putNumber("Drive kD", 0);
    */

    Shuffleboard.selectTab("Teleop");
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Shuffleboard.getTab("Teleop").addNumber("m_speedy", new DoubleSupplier(){
      @Override
      public double getAsDouble() {
        return m_robotContainer.m_speedy;
      }
    });
    Shuffleboard.getTab("Teleop").addString("m_swappy", new Supplier<String>(){
      @Override
      public String get() {
        if (m_robotContainer.m_swappy == true) {
          return "Inverted";
        } else {
          return "Normal";
        }
      }
    });
    Shuffleboard.getTab("Teleop").addString("Compressor", new Supplier<String>(){
      @Override
      public String get() {
        if (m_robotContainer.getCompressor().enabled() == true) {
          return "ON";
        } else {
          return "OFF";
        }
      }
    });
    /*
    VisionConstants.kpDistance = SmartDashboard.getNumber("Drive kP", 0);
    VisionConstants.kiDistance = SmartDashboard.getNumber("Drive kI", 0);
    VisionConstants.kdDistance = SmartDashboard.getNumber("Drive kD", 0);
    */
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.getGyro().reset();
    m_robotContainer.getVisionSystem().setPipeline(1);

    Shuffleboard.selectTab("Auto");
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    
  }
}
