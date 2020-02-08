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

public class ShooterSystem extends PIDSubsystem {

  private final CANSparkMax m_topMotor, m_bottomMotor;
  private double m_topMotorSpeed, m_bottomMotorSpeed;

  public ShooterSystem() {
    super(new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD));

    m_topMotor = new CANSparkMax(ShooterConstants.shooterFirstPort, MotorType.kBrushless);
    m_bottomMotor = new CANSparkMax(ShooterConstants.shooterSecondPort, MotorType.kBrushless);

    m_topMotor.setInverted(false);
    m_bottomMotor.setInverted(false);

    m_topMotor.getEncoder().setPosition(0);
    m_bottomMotor.getEncoder().setPosition(0);

    m_topMotorSpeed = 0;
    m_bottomMotorSpeed = 0;
  }

  public void spinShooter(double motorRPM) {
    if (m_topMotor.getEncoder().getVelocity() < motorRPM) {
      m_topMotorSpeed += 0.001;
      m_topMotor.set(m_topMotorSpeed);
    } else if (m_topMotor.getEncoder().getVelocity() > motorRPM) {
      m_topMotorSpeed -= 0.001;
      m_topMotor.set(m_topMotorSpeed);
    }

    if (m_bottomMotor.getEncoder().getVelocity() < motorRPM) {
      m_bottomMotorSpeed += 0.001;
      m_bottomMotor.set(m_bottomMotorSpeed);
    } else if (m_bottomMotor.getEncoder().getVelocity() > motorRPM) {
      m_bottomMotorSpeed -= 0.001;
      m_bottomMotor.set(m_bottomMotorSpeed);
    }
  }

  public void manualSpinMotor(double speed) {
    m_topMotor.set(speed);
    m_bottomMotor.set(speed);
  }

  public void reset() {
    disable();
    m_topMotor.stopMotor();
    m_bottomMotor.stopMotor();
    m_topMotor.getEncoder().setPosition(0);
    m_bottomMotor.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  protected void useOutput(double output, double setpoint) {
    m_topMotor.set(output);
    m_bottomMotor.set(output);
  }

  @Override
  protected double getMeasurement() {
    return m_topMotor.getEncoder().getVelocity();
  }
}
