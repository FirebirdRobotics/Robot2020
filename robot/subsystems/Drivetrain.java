/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
  /**
   * Creates a new Drivetrain.
   */

  private final WPI_TalonFX m_leftMaster, m_rightMaster, m_leftSlave, m_rightSlave;

  private final SupplyCurrentLimitConfiguration m_currentLimitConfig;

  private final DifferentialDrive m_diffDrive;

  public Drivetrain() {
    m_leftMaster = new WPI_TalonFX(DriveConstants.dtFrontLeftPort);
    m_rightMaster = new WPI_TalonFX(DriveConstants.dtFrontRightPort);
    m_leftSlave = new WPI_TalonFX(DriveConstants.dtBackLeftPort);
    m_rightSlave = new WPI_TalonFX(DriveConstants.dtBackRightPort);

    m_currentLimitConfig = new SupplyCurrentLimitConfiguration(
      DriveConstants.kCurrentLimitingEnabled, DriveConstants.kPeakCurrentAmps,
      DriveConstants.kContCurrentAmps, DriveConstants.kPeakTimeMs);

    m_diffDrive = new DifferentialDrive(m_leftMaster, m_rightMaster);

    configTalon(m_leftMaster);
    configTalon(m_rightMaster);
    configTalon(m_leftSlave);
    configTalon(m_rightSlave);

    m_leftSlave.follow(m_leftMaster);
    m_rightSlave.follow(m_rightMaster);

    m_leftMaster.setInverted(false); // CHANGE THESE UNTIL ROBOT DRIVES FORWARD
    m_rightMaster.setInverted(true); // CHANGE THESE UNTIL ROBOT DRIVES FORWARD
    m_leftSlave.setInverted(InvertType.FollowMaster);
    m_rightSlave.setInverted(InvertType.FollowMaster);

    // diffdrive assumes by default that right side must be negative- change to false for master/slave config
    m_diffDrive.setRightSideInverted(false); // DO NOT CHANGE THIS

    // deadband: if you accidentally move the joystick a small amnt, it won't move if within deadband
    m_diffDrive.setDeadband(DriveConstants.kDeadband);
  }

  public void configTalon(WPI_TalonFX m_talon) {
    // factory reset hardware to make sure nothing unexpected
    m_talon.configFactoryDefault();

    // use the defined SupplyCurrentLimitConfiguration object to limit the current of the motors
    m_talon.configSupplyCurrentLimit(m_currentLimitConfig, DriveConstants.kTimeoutMs);
  }

  public void arcadeDrive(double forward, double turn) {
    m_diffDrive.arcadeDrive(forward * DriveConstants.kDriveSpeed, turn * DriveConstants.kTurnSpeed, true);
  }

  public void curvatureDrive(double forward, double turn, boolean isQuickTurn) {
    m_diffDrive.curvatureDrive(forward * DriveConstants.kDriveSpeed, turn * DriveConstants.kTurnSpeed, isQuickTurn);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}