/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSystem extends SubsystemBase {
  
  private final CANSparkMax m_leftElevator, m_rightElevator, m_skiLift;

  public ClimbSystem() {
    m_leftElevator = new CANSparkMax(ClimbConstants.elevatorFirstPort, MotorType.kBrushless);
    m_rightElevator = new CANSparkMax(ClimbConstants.elevatorSecondPort, MotorType.kBrushless);
    m_skiLift = new CANSparkMax(ClimbConstants.skiLiftPort, MotorType.kBrushless);

    m_leftElevator.getEncoder().setPosition(0);
    m_rightElevator.getEncoder().setPosition(0);
    m_skiLift.getEncoder().setPosition(0);

    m_leftElevator.setInverted(false);
    m_rightElevator.setInverted(false);
    m_skiLift.setInverted(false);
  }

  public void setElevatorPosition(double position) {
    m_leftElevator.getEncoder().setPosition(position);
    m_rightElevator.getEncoder().setPosition(position);
  }

  public void moveSkiLift(double speed) {
    m_skiLift.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
