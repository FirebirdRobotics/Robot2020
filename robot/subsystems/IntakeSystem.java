/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSystem extends SubsystemBase {
  
  private final CANSparkMax m_intakeMotor;
  private final Solenoid m_intakeSolenoid;

  public IntakeSystem() {
    m_intakeMotor = new CANSparkMax(IntakeConstants.intakePort, MotorType.kBrushless);
    m_intakeSolenoid = new Solenoid(IntakeConstants.intakeSolenoidRight);

  }

  public void setIntake(double speed) {
    m_intakeMotor.set(speed);
  }

  // no idea what this function requires right now, hopefully its pneumatic
  public void retractIntake() {
    m_intakeSolenoid.set(!m_intakeSolenoid.get());
  }

  public void updateDashboard() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
