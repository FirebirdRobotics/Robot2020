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
  
  private final CANSparkMax m_elevatorMotor, m_skiLift;

  public ClimbSystem() {
    m_elevatorMotor = new CANSparkMax(ClimbConstants.elevatorPort, MotorType.kBrushless);
    m_skiLift = new CANSparkMax(ClimbConstants.skiLiftPort, MotorType.kBrushless);

    m_elevatorMotor.getEncoder().setPosition(0);
    m_skiLift.getEncoder().setPosition(0);

    m_elevatorMotor.setInverted(false);
    m_skiLift.setInverted(false);

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

  public void stopElevator() {
    m_elevatorMotor.set(0);
  }

  public void moveSkiLift(double speed) {
    m_skiLift.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
