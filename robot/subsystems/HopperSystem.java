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
import frc.robot.Constants.HopperConstants;

public class HopperSystem extends SubsystemBase {
  
  private final CANSparkMax m_hopperMotor;
  private final Solenoid m_hopperPiston;

  public HopperSystem() {
    m_hopperMotor = new CANSparkMax(HopperConstants.hopperPort, MotorType.kBrushless);
    m_hopperPiston = new Solenoid(HopperConstants.hopperSolenoid);
  }

  public void runHopper(double speed) {
    m_hopperMotor.set(speed);
  }

  // using a single solenoid as double solenoid
  public void toggleSolenoid() {
    m_hopperPiston.set(!m_hopperPiston.get());
  }

  public boolean empty() {
    return true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
