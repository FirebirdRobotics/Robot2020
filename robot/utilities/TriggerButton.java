/*----------------------------------------------------------------------------*/
/* Copyright (c) 2008-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utilities;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

/**
 * A {@link Trigger} that gets its state from a {@link GenericHID}.
 */
public class TriggerButton extends Trigger{
  private final GenericHID m_joystick;
  private final int m_buttonNumber;

  /**
   * Creates a joystick trigger for triggering commands.
   *
   * @param joystick     The GenericHID object that has the button (e.g. Joystick, KinectStick,
   *                     etc)
   * @param triggerNumber The trigger number (see {@link GenericHID#getRawButton(int) }
   */
  public TriggerButton(GenericHID joystick, int triggerNumber) {
    requireNonNullParam(joystick, "joystick", "JoystickButton");

    m_joystick = joystick;
    m_buttonNumber = triggerNumber;
  }

  /**
   * Gets the value of the joystick trigger.
   *
   * @return The value of the joystick trigger
   */
  @Override
  public boolean get() {
    return m_joystick.getRawButton(m_buttonNumber);
  }
}
