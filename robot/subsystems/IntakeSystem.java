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
import frc.robot.Constants.IntakeConstants;

public class IntakeSystem extends SubsystemBase {
  
  private final CANSparkMax m_intakeLeft, m_intakeRight;

  public IntakeSystem() {
    m_intakeLeft = new CANSparkMax(IntakeConstants.intakeLeftPort, MotorType.kBrushless);
    m_intakeRight = new CANSparkMax(IntakeConstants.intakeRightPort, MotorType.kBrushless);

  }

  public void runIntake(double speed) {
    m_intakeLeft.set(speed);
    m_intakeRight.set(speed);
  }

  // no idea what this function requires right now, hopefully its pneumatic
  public void retractIntake() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
