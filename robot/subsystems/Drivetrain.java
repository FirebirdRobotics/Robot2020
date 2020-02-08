/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

//import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    // flip so that motor output and sensor velocity are same polarity
    m_leftMaster.setSensorPhase(false);
    m_rightMaster.setSensorPhase(true);
    m_leftSlave.setSensorPhase(false);
    m_rightSlave.setSensorPhase(true);

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

    // setup encoders; talonFX integrated sensor, closed loop
    m_talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, DriveConstants.kTimeoutMs);
  }

  public void arcadeDrive(double forward, double turn) {
    m_diffDrive.arcadeDrive(-forward * DriveConstants.kDriveSpeed, turn * DriveConstants.kTurnSpeed, true);
    
    updateDashboard(m_leftMaster, "Left Master");
    updateDashboard(m_rightMaster, "Right Master");
    updateDashboard(m_leftSlave, "Left Slave");
    updateDashboard(m_rightSlave, "Right Slave");
  }

  public void curvatureDrive(double forward, double turn, boolean isQuickTurn) {
    m_diffDrive.curvatureDrive(-forward * DriveConstants.kDriveSpeed, turn * DriveConstants.kTurnSpeed, isQuickTurn);

    updateDashboard(m_leftMaster, "Left Master");
    updateDashboard(m_rightMaster, "Right Master");
    updateDashboard(m_leftSlave, "Left Slave");
    updateDashboard(m_rightSlave, "Right Slave");
  }

  public void updateDashboard(WPI_TalonFX talon, String talonName) {
    SmartDashboard.putNumber("Output % (" + talonName + ")", talon.getMotorOutputPercent());
    SmartDashboard.putNumber("Sensor Pos. (" + talonName + ")", talon.getSelectedSensorPosition());
    // SmartDashboard.putNumber("Sensor Vel. (" + talonName + ")", talon.getSelectedSensorVelocity());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}