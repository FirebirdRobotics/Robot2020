/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSystem extends SubsystemBase {
  /**
   * Creates a new New_ShooterSystem.
   */
  private final CANSparkMax m_master, m_slave;
  private final ShuffleboardTab m_teleopTab = Shuffleboard.getTab("Teleop");


  public ShooterSystem() {
    m_master = new CANSparkMax(ShooterConstants.shooterFirstPort, MotorType.kBrushless);
    m_slave = new CANSparkMax(ShooterConstants.shooterSecondPort, MotorType.kBrushless);

    m_master.getEncoder().setPosition(0);
    m_slave.getEncoder().setPosition(0);

    m_slave.follow(m_master, true);

    m_master.setInverted(false);
  }

  public void manualSpinMotor(double speed) {
    m_master.set(speed);
  }

  public double getSpeed() {
    return m_master.getEncoder().getVelocity() / MotorConstants.kNeoRPM;
  }

  public void updateDashboard() {
    m_teleopTab.addNumber("Shooter Speed (ratio)", new DoubleSupplier(){
      @Override
      public double getAsDouble() {
        return getSpeed();
      }
    });
    m_teleopTab.addNumber("Current Shooter RPM", new DoubleSupplier(){
      @Override
      public double getAsDouble() {
        return getSpeed() * MotorConstants.kNeoRPM;
      }
    });
  }

  /**
   * 
   * @param targetSpeed The speed, in ratio form, at which to compare the current speed of the motors in this subsystem.
   */
  public void updateDashboard(double targetSpeed) {
    updateDashboard();
    m_teleopTab.addNumber("Shooter Spin Efficiency", new DoubleSupplier(){
      @Override
      public double getAsDouble() {
        return getSpeed() *100 / targetSpeed;
      }
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}