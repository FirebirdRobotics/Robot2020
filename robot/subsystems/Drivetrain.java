/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MotorConstants;

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

    m_leftMaster.setInverted(true); // CHANGE THESE UNTIL ROBOT DRIVES FORWARD
    m_rightMaster.setInverted(false); // CHANGE THESE UNTIL ROBOT DRIVES FORWARD
    m_leftSlave.setInverted(InvertType.FollowMaster);
    m_rightSlave.setInverted(InvertType.FollowMaster);

    // flip so that motor output and sensor velocity are same polarity
    m_leftMaster.setSensorPhase(false);
    m_rightMaster.setSensorPhase(false);
    m_leftSlave.setSensorPhase(false);
    m_rightSlave.setSensorPhase(false);

    // set mode of motors
    setNeutralMode(NeutralMode.Coast);

    // diffdrive assumes by default that right side must be negative- change to false for master/slave config
    m_diffDrive.setRightSideInverted(false); // DO NOT CHANGE THIS

    // deadband: motors wont move if speed of motors is within deadband
    m_diffDrive.setDeadband(DriveConstants.kDeadband);
  }

  // ARCADE DRIVE = 1 STICK
  public void arcadeDrive(final double forward, final double turn) {
    m_diffDrive.arcadeDrive(forward * DriveConstants.kDriveSpeed, turn * DriveConstants.kTurnSpeed, true);
    
    updateDashboard(m_leftMaster, "Left Master");
    updateDashboard(m_rightMaster, "Right Master");
    updateDashboard(m_leftSlave, "Left Slave");
    updateDashboard(m_rightSlave, "Right Slave");
  }

  // CURVATURE DRIVE = 2 STICK + QUICK TURN BUTTON
  public void curvatureDrive(final double forward, final double turn, final boolean isQuickTurn) {
    m_diffDrive.curvatureDrive(forward * DriveConstants.kDriveSpeed, turn * DriveConstants.kTurnSpeed, isQuickTurn);

    updateDashboard(m_leftMaster, "Left Master");
    updateDashboard(m_rightMaster, "Right Master");
    updateDashboard(m_leftSlave, "Left Slave");
    updateDashboard(m_rightSlave, "Right Slave");
  }

  // AUTO DRIVE = for autonomous commands
  public void autoDrive(final double forward, final double turn) {
    m_diffDrive.arcadeDrive(forward, turn, false);
  }

  public void configTalon(final WPI_TalonFX m_talon) {
    // factory reset hardware to make sure nothing unexpected
    m_talon.configFactoryDefault();

    // use the defined SupplyCurrentLimitConfiguration object to limit the current of the motors
    m_talon.configSupplyCurrentLimit(m_currentLimitConfig, DriveConstants.kTimeoutMs);

    // setup encoders; talonFX integrated sensor, closed loop
    m_talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, DriveConstants.kTimeoutMs);
    m_talon.setSelectedSensorPosition(0); // resets encoder counts
  }

  public void resetEncoders() {
    m_rightMaster.setSelectedSensorPosition(0);
    m_rightSlave.setSelectedSensorPosition(0);
    m_leftMaster.setSelectedSensorPosition(0);
    m_leftSlave.setSelectedSensorPosition(0);
  }

  // autonomously drive using encoder counts (send in inches)
  public void driveWithEncoders(final double targetEncoderCounts) {
    final double targetZoneLower = targetEncoderCounts - (targetEncoderCounts * 0.05);
    final double targetZoneUpper = targetEncoderCounts + (targetEncoderCounts * 0.05);

    // just measure one of the master motors and then run entire DT
    if (m_rightMaster.getSelectedSensorPosition() < targetZoneLower) {
      System.out.println("target encoder counts: " + targetEncoderCounts);
      System.out.println("encoder: " + m_rightMaster.getSelectedSensorPosition());
      this.autoDrive(0.2, 0);
    } else if (m_rightMaster.getSelectedSensorPosition() > targetZoneUpper) {
      System.out.println("target encoder counts: " + targetEncoderCounts);
      this.autoDrive(-0.2, 0);
    } else {
      this.autoDrive(0, 0);
    }
  }

  // returns true when done
  public boolean doneDrivingEncoder(final double targetEncoderCounts) {
    final double targetZoneLower = targetEncoderCounts - (targetEncoderCounts * DriveConstants.kDriveDistanceError);
    final double targetZoneUpper = targetEncoderCounts + (targetEncoderCounts * DriveConstants.kDriveDistanceError);

    if (m_rightMaster.getSelectedSensorPosition() <= targetZoneLower) {
      return false;
    } else if (m_rightMaster.getSelectedSensorPosition() >= targetZoneUpper) {
      return false;
    } else { 
      return true;
    }
  }

  // set neutral mode
  public void setNeutralMode(final NeutralMode neutralMode) {
		m_leftMaster.setNeutralMode(neutralMode);
		m_leftSlave.setNeutralMode(neutralMode);
		m_rightMaster.setNeutralMode(neutralMode);
		m_rightSlave.setNeutralMode(neutralMode);
  }
  
  public WPI_TalonFX[] getMotors() {
    WPI_TalonFX[] motors = {m_leftMaster, m_leftSlave, m_rightMaster, m_rightSlave};

    return motors;
  }

  // returns RPM
  public double getVelocity() {
    return (m_rightMaster.getSelectedSensorVelocity() * 10 * 60) / (MotorConstants.kFalconCPR);
  }

  public void updateDashboard(final WPI_TalonFX talon, final String talonName) {
    SmartDashboard.putNumber("Output % (" + talonName + ")", talon.getMotorOutputPercent());
    SmartDashboard.putNumber("Sensor Pos. (" + talonName + ")", talon.getSelectedSensorPosition());
    // SmartDashboard.putNumber("Sensor Vel. (" + talonName + ")", talon.getSelectedSensorVelocity());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}