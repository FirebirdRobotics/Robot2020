/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.MotorConstants;

public class ShooterSystem extends PIDSubsystem {
  private final CANSparkMax m_master, m_slave;
  private double m_motorSpeed;

  public ShooterSystem() {
    super(new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD));

    m_master = new CANSparkMax(ShooterConstants.shooterFirstPort, MotorType.kBrushless);
    m_slave = new CANSparkMax(ShooterConstants.shooterSecondPort, MotorType.kBrushless);

    m_master.getEncoder().setPosition(0);
    m_slave.getEncoder().setPosition(0);

    m_slave.follow(m_master, true);

    m_master.setInverted(false);

    m_motorSpeed = 0;
  }

  public void spinShooter(double motorRPM) {
    if (m_master.getEncoder().getVelocity() < motorRPM) {
      m_motorSpeed += 0.001;
      m_master.set(m_motorSpeed);
    } else if (m_master.getEncoder().getVelocity() > motorRPM) {
      m_motorSpeed -= 0.001;
      m_master.set(m_motorSpeed);
    }
  }

  public void manualSpinMotor(double speed) {
    m_master.set(speed);
  }

  public void reset() {
    disable();
    m_master.stopMotor();
    m_master.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  protected void useOutput(double output, double setpoint) {
    m_master.set(output);
  }

  @Override
  protected double getMeasurement() {
    return m_master.getEncoder().getVelocity() / MotorConstants.kNeoRPM;
  }
}
