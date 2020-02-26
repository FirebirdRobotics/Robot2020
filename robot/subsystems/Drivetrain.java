/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

// import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

// import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.UnitConversionConstants;

public class Drivetrain extends SubsystemBase {
  /**
   * Creates a new Drivetrain.
   */

  private final WPI_TalonFX m_leftMaster, m_rightMaster, m_leftSlave, m_rightSlave;

  private final SupplyCurrentLimitConfiguration m_currentLimitConfig;

  private final DifferentialDrive m_diffDrive;

  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  private ShuffleboardTab m_autoTab = Shuffleboard.getTab("Auto");

  private final DifferentialDriveOdometry m_odometry;

  private boolean dtIsInverted;

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

    dtIsInverted = false;

    // flip so that motor output and sensor velocity are same polarity
    m_leftMaster.setSensorPhase(false);
    m_rightMaster.setSensorPhase(false);
    m_leftSlave.setSensorPhase(false);
    m_rightSlave.setSensorPhase(false);

    // set mode of motors
    setNeutralMode(DriveConstants.kMotorMode);

    // diffdrive assumes by default that right side must be negative- change to false for master/slave config
    m_diffDrive.setRightSideInverted(false); // DO NOT CHANGE THIS

    // deadband: motors wont move if speed of motors is within deadband
    m_diffDrive.setDeadband(DriveConstants.kDeadband);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
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

  /**
	 * Controls the left and right sides of the drive directly with voltages.
	 *
	 * @param leftVolts The voltage to output for the left side
	 * @param rightVolts The voltage to output for the right side
	 */
	public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMaster.setVoltage(leftVolts);
    m_rightMaster.setVoltage(rightVolts);
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
    final double targetZoneLower = targetEncoderCounts - (targetEncoderCounts * DriveConstants.kDriveDistanceError);
    final double targetZoneUpper = targetEncoderCounts + (targetEncoderCounts * DriveConstants.kDriveDistanceError);

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

  /**
   * Sets the inversion state of the drivetrain.
   * @param setInverted Set true to invert Drivetrain
   */
  public void setDrivetrainInverted(boolean setInverted) {
    if (setInverted && !dtIsInverted) { // if pass in TRUE and DT is not inverted
      m_leftMaster.setInverted(false);
      m_rightMaster.setInverted(true);
      dtIsInverted = true;
    }
    else if (!setInverted && dtIsInverted) { // if pass in FALSE and DT is inverted
      m_leftMaster.setInverted(true);
      m_rightMaster.setInverted(false);
      dtIsInverted = false;
    }
  }

  /**
	 * Returns the heading of the robot in form required for odometry.
	 *
	 * @return the robot's heading in degrees, from 180 to 180 with positive value
	 *         for left turn.
	 */
	public double getHeading() {
		return m_gyro.getAngle();
  }
  
  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
	 * Returns left encoder position in meters
	 * 
	 * @return left encoder position (in meters)
	 */
	public double getLeftEncoderPosition() {
    // get rotations of encoder by dividing encoder counts by counts per rotation
    double encoderRotations = m_leftMaster.getSelectedSensorPosition() / MotorConstants.kFalconCPR;

    // get rotations of wheel by diving rotations of encoder by gear ratio
    double wheelRotations = encoderRotations / DriveConstants.kGearRatio;

    // get distance by multiplying rotations of wheel by circumference of wheel (2 * pi * radius)
    double distance = wheelRotations * (2 * Math.PI * (DriveConstants.kWheelRadius * UnitConversionConstants.distanceConversionFactor));

		return distance;
	}

	/**
	 * Returns right encoder position in meters
	 * 
	 * @return right encoder position (in meters)
	 */
	public double getRightEncoderPosition() {
    // get rotations of encoder by dividing encoder counts by counts per rotation
    double encoderRotations = m_rightMaster.getSelectedSensorPosition() / MotorConstants.kFalconCPR;

    // get rotations of wheel by diving rotations of encoder by gear ratio
    double wheelRotations = encoderRotations / DriveConstants.kGearRatio;

    // get distance by multiplying rotations of wheel by circumference of wheel (2 * pi * radius)
    double distance = wheelRotations * (2 * Math.PI * (DriveConstants.kWheelRadius * UnitConversionConstants.distanceConversionFactor));

		return distance;
  }

  /**
	 * Get the left encoder velocity in meters per second
	 * 
	 * @return Get the left encoder velocity (in m/s)
	 */
	public double getLeftEncoderRate() {
    // get rotations of encoder by dividing encoder counts by counts per rotation
    double encoderRotations = m_leftMaster.getSelectedSensorVelocity() / MotorConstants.kFalconCPR;

    // get rotations of wheel by diving rotations of encoder by gear ratio
    double wheelRotations = encoderRotations / DriveConstants.kGearRatio;

    // get velocity by multiplying rotations of wheel by circumference of wheel (2 * pi * radius)
    double velocity = wheelRotations * (2 * Math.PI * (DriveConstants.kWheelRadius * UnitConversionConstants.distanceConversionFactor));

    // get distance per second by multiplying by 10; getSelectedSensorVelocity sends distance back per 100ms
    velocity *= 10;

		return velocity;
	}

	/**
	 * Get the right encoder velocity in meters per second
	 * 
	 * @return Get the right encoder velocity (in m/s)
	 */
	public double getRightEncoderRate() {
		// get rotations of encoder by dividing encoder counts by counts per rotation
    double encoderRotations = m_rightMaster.getSelectedSensorVelocity() / MotorConstants.kFalconCPR;

    // get rotations of wheel by diving rotations of encoder by gear ratio
    double wheelRotations = encoderRotations / DriveConstants.kGearRatio;

    // get velocity by multiplying rotations of wheel by circumference of wheel (2 * pi * radius)
    double velocity = wheelRotations * (2 * Math.PI * (DriveConstants.kWheelRadius * UnitConversionConstants.distanceConversionFactor));

    // get distance per second by multiplying by 10; getSelectedSensorVelocity sends distance back per 100ms
    velocity *= 10;

    return velocity;
	}
  
  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderRate(), getRightEncoderRate()); // we are sending in meters/second
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (getLeftEncoderPosition() + getRightEncoderPosition()) / 2.0;
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_gyro.reset();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  public void updateDashboard(final WPI_TalonFX talon, final String talonName) {
    SmartDashboard.putNumber("Output % (" + talonName + ")", talon.getMotorOutputPercent());
    SmartDashboard.putNumber("Sensor Pos. (" + talonName + ")", talon.getSelectedSensorPosition());
  }

  // Returns in a ratio unit depending on the motor's max RPM.
  public double getVelocity () {
    return ((m_rightMaster.getSelectedSensorVelocity() + m_leftMaster.getSelectedSensorVelocity()) * 5 / (MotorConstants.kFalconCPR * MotorConstants.kFalconRPM));
  }

  public double getRightEncoderPosition2 () {
    return m_rightMaster.getSelectedSensorPosition();
  }

  public void updateDashboard() {
    m_autoTab.add("Pose", m_odometry.getPoseMeters().toString());

		m_autoTab.addNumber("Left Position", new DoubleSupplier() {
			@Override
			public double getAsDouble() {
				return getLeftEncoderPosition();
			}
    });
    
    m_autoTab.addNumber("Right Position", new DoubleSupplier(){
			@Override
			public double getAsDouble() {
				return getRightEncoderPosition();
			}
		});

		m_autoTab.addNumber("Left Velocity", new DoubleSupplier() {
			@Override
			public double getAsDouble() {
				return getLeftEncoderRate();
			}
    });
    
    m_autoTab.addNumber("Right Velocity", new DoubleSupplier() {
			@Override
			public double getAsDouble() {
				return getRightEncoderRate();
			}
		});

		m_autoTab.addNumber("Left Volts", new DoubleSupplier() {
			@Override
			public double getAsDouble() {
				return m_leftMaster.getMotorOutputVoltage();
			}
		}).withWidget("Graph");

		m_autoTab.addNumber("Right Volts", new DoubleSupplier() {
			@Override
			public double getAsDouble() {
				return m_rightMaster.getMotorOutputVoltage();
			}
    }).withWidget("Graph");
    
    m_autoTab.addNumber("Left Setpoint", new DoubleSupplier() {
			@Override
			public double getAsDouble() {
				return AutonomousConstants.leftController.getSetpoint();
			}
		}).withWidget("Graph");

		m_autoTab.addNumber("Right Setpoint", new DoubleSupplier() {
			@Override
			public double getAsDouble() {
				return AutonomousConstants.rightController.getSetpoint();
			}
		}).withWidget("Graph");
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftEncoderPosition(), getRightEncoderPosition());
    updateDashboard();
  }
}