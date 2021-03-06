/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Solenoids extends SubsystemBase {

  public Solenoids() {

  }

  // using a single solenoid as double solenoid
  public void toggleSolenoid(Solenoid solenoid) {
    solenoid.set(!solenoid.get());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
