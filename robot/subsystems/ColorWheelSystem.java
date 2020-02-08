/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

//import javax.swing.text.StyleConstants.ColorConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ColorWheelConstants;

public class ColorWheelSystem extends SubsystemBase {
  
  private final CANSparkMax m_colorSpinner;
  private final ColorSensorV3 m_colorSensor;
  private final ColorMatch m_colorMatcher;

  public ColorWheelSystem() {
    m_colorSpinner = new CANSparkMax(ColorWheelConstants.colorWheelPort, MotorType.kBrushless);
    m_colorSpinner.getEncoder().setPosition(0);

    m_colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    m_colorMatcher = new ColorMatch();

    m_colorMatcher.addColorMatch(ColorWheelConstants.kBlueTarget);
    m_colorMatcher.addColorMatch(ColorWheelConstants.kGreenTarget);
    m_colorMatcher.addColorMatch(ColorWheelConstants.kRedTarget);
    m_colorMatcher.addColorMatch(ColorWheelConstants.kYellowTarget);
  }

  public void spinColorWheel(double speed) {
    m_colorSpinner.set(speed);
  }

  public Color getCurrentColor() {
    Color detectedColor = m_colorSensor.getColor();
    ColorMatchResult closestMatch = m_colorMatcher.matchClosestColor(detectedColor);

    String colorString;
    if (closestMatch.color == ColorWheelConstants.kBlueTarget) {
      colorString = "Blue";
    } else if (closestMatch.color == ColorWheelConstants.kRedTarget) {
      colorString = "Red";
    } else if (closestMatch.color == ColorWheelConstants.kGreenTarget) {
      colorString = "Green";
    } else if (closestMatch.color == ColorWheelConstants.kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }

    SmartDashboard.putString("Color Detected", colorString);

    return closestMatch.color;
  }

  public void spinToColor(Color desiredColor, double speed) {
    if (getCurrentColor() != desiredColor) {
      spinColorWheel(speed);
    } else if (getCurrentColor() == desiredColor) {
      spinColorWheel(0.0);
    }

  }

  public void spinOneRotation() {

  }

  // will send in a sendable chooser (select color from dashboard)
  public void spinToSpecifiedColor() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
