/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

/**
 * For possibly using SmartMotion: 
 * https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Smart%20Motion%20Example/src/main/java/frc/robot/Robot.java
 */

public class ClimbSystem extends SubsystemBase {
  
  private final CANSparkMax m_elevatorMotor, m_skiLift;
  private final CANPIDController m_elevatorPID;

  public ClimbSystem() {
    m_elevatorMotor = new CANSparkMax(ClimbConstants.elevatorPort, MotorType.kBrushless);
    m_skiLift = new CANSparkMax(ClimbConstants.skiLiftPort, MotorType.kBrushless);

    m_elevatorMotor.restoreFactoryDefaults();

    m_elevatorMotor.getEncoder().setPosition(0);
    m_skiLift.getEncoder().setPosition(0);

    m_elevatorMotor.setInverted(false);
    m_skiLift.setInverted(false);

    // setup elevator PID
    m_elevatorPID = m_elevatorMotor.getPIDController();

    // set PID coefficients
    m_elevatorPID.setP(ClimbConstants.kP);
    m_elevatorPID.setI(ClimbConstants.kI);
    m_elevatorPID.setD(ClimbConstants.kD);
    m_elevatorPID.setIZone(ClimbConstants.kIZ);
    m_elevatorPID.setFF(ClimbConstants.kFF);
    m_elevatorPID.setOutputRange(ClimbConstants.kMinOutput, ClimbConstants.kMaxOutput);

    m_elevatorPID.setSmartMotionMaxVelocity(ClimbConstants.kMaxVel, 0);
    m_elevatorPID.setSmartMotionMinOutputVelocity(ClimbConstants.kMinVel, 0);
    m_elevatorPID.setSmartMotionMaxAccel(ClimbConstants.kMaxAccel, 0);
    m_elevatorPID.setSmartMotionAllowedClosedLoopError(ClimbConstants.kAllowedError, 0);

    SmartDashboard.putBoolean("Mode", true);
  }

  // set to encoder count, return boolean true when done
  public boolean setElevatorPosition(double position) {
    if (position < m_elevatorMotor.getEncoder().getPosition()) {
      m_elevatorMotor.set(ClimbConstants.kElevatorSpeed);
      return false;
    } else if (position > m_elevatorMotor.getEncoder().getPosition()) {
      m_elevatorMotor.set(-ClimbConstants.kElevatorSpeed);
      return false;
    } else {
      stopElevator();
      return true;
    }
  }

  /**
   * Uses REV's smart motion to control the elevator's velocity.
   * @param setpoint The target velocity of the elevator. 
   * @param on Whether or not you want smart motion on
   */
  public void setSmartMotion(double setpoint, boolean on) {
    if (on) {
      m_elevatorPID.setReference(setpoint, ControlType.kSmartMotion);
    } else if (!on) {
      m_elevatorPID.setReference(setpoint, ControlType.kVelocity);
    }
  }

  public void stopElevator() {
    m_elevatorMotor.set(0);
  }

  public void moveSkiLift(double speed) {
    m_skiLift.set(speed);
  }

  public void updateDashboard() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
