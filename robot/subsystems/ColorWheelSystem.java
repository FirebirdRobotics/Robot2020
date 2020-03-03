/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ColorWheelConstants;

public class ColorWheelSystem extends SubsystemBase {
  
  private final CANSparkMax m_colorSpinner;
  private final ColorSensorV3 m_colorSensor;
  private final ColorMatch m_colorMatcher;
  private String colorString;

  private final ShuffleboardTab m_teleopTab = Shuffleboard.getTab("Teleop");

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

  public void setColorSpinner(double speed) {
    m_colorSpinner.set(speed);
  }

  public Color getCurrentColor() {
    Color detectedColor = m_colorSensor.getColor();
    ColorMatchResult closestMatch = m_colorMatcher.matchClosestColor(detectedColor);

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

    updateDashboard();

    return closestMatch.color;
  }

  /**
   * Spins the color wheel to a certain color
   * @param desiredColor The color to spin to; recommend using a sendable chooser
   */
  public void spinToColor(Color desiredColor) {
    if (getCurrentColor() != desiredColor) {
      setColorSpinner(ColorWheelConstants.kColorSpinnerSpeed);
    } else if (getCurrentColor() == desiredColor) {
      setColorSpinner(0.0);
    }

  }

  /**
   * Use encoder counts to spin a number of rotations
   * @param rotations Number of rotations to spin
   */
  public void spinRotations(int rotations) {
    double encoderPosition = m_colorSpinner.getEncoder().getPosition();
    if (encoderPosition < (ColorWheelConstants.kCountsPerRotation * rotations)) {
      setColorSpinner(ColorWheelConstants.kColorSpinnerSpeed);
    } else if (encoderPosition >= (ColorWheelConstants.kCountsPerRotation * rotations)) {
      setColorSpinner(0.0);
    }
    
  }

  public void updateDashboard() {
    m_teleopTab.addString("Color Detected", new Supplier<String>(){
      @Override
      public String get() {
        return colorString;
      }
    });
  }

  @Override
  public void periodic() {
    updateDashboard();
  }
}
