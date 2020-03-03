/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;

public class HopperSystem extends SubsystemBase {
  
  private final CANSparkMax m_hopperMotor;
  private final Solenoid m_hopperPiston;

  private final ShuffleboardTab m_teleopTab = Shuffleboard.getTab("Teleop");

  public HopperSystem() {
    m_hopperMotor = new CANSparkMax(HopperConstants.hopperPort, MotorType.kBrushless);
    m_hopperPiston = new Solenoid(HopperConstants.hopperSolenoid);
  }

  public void setHopper(double speed) {
    m_hopperMotor.set(speed);
  }

  // using a single solenoid as double solenoid
  public void toggleSolenoid() {
    m_hopperPiston.set(!m_hopperPiston.get());
  }

  /**
   * Sets state of the solenoid in the hopper.
   * @param state True sets piston out (closed), false sets piston in (open).
   */
  public void setSolenoid(boolean state) {
    if (state && !m_hopperPiston.get()) {
      m_hopperPiston.set(state);
    }
    else if (!state && m_hopperPiston.get()) {
      m_hopperPiston.set(state);
    }
  }

  public boolean empty() {
    return true;
  }

  public void updateDashboard() {
    m_teleopTab.addString("Hopper", new Supplier<String>(){
      @Override
      public String get() {
        if (m_hopperPiston.get()) {
          return "CLOSED";
        } else {
          return "OPEN";
        }
      }
    });
  }

  @Override
  public void periodic() {
    updateDashboard();
  }
}
