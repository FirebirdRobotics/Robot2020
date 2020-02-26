/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSystem extends SubsystemBase {
  
  private Spark m_revBlinkin;

  private int m_colorSelection;

  private double[] m_colors = {
    LEDConstants.kRed,
    LEDConstants.kYellow,
    LEDConstants.kBlue,
    LEDConstants.kWhite,
    LEDConstants.kRainbowLavaPalette,
    LEDConstants.kSinelonLavaPalette,
    LEDConstants.kBeatsPerMinuteLavaPalette,
    LEDConstants.kFireMedium,
    LEDConstants.kFireLarge,
    LEDConstants.kTwinklesLavaPalette,
    LEDConstants.kColorWavesLavaPalette,
    LEDConstants.kLarsonScannerRed,
    LEDConstants.kLightChaseRed,
    LEDConstants.kHeartbeatRed,
    LEDConstants.kBreatheRed,
    LEDConstants.kStrobeRed,
    LEDConstants.kRainbow
  };

  private ShuffleboardTab m_autoTab = Shuffleboard.getTab("Auto");

  public LEDSystem() {
    m_revBlinkin = new Spark(LEDConstants.revBlinkinPort);
  }

  // Go to page 14, hopefully you have internet: http://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf
  public void setColor(double color) {
    m_revBlinkin.set(color);
  }

  public String getCurrentColorString() {
    if (m_revBlinkin.get() == LEDConstants.kRed) {
      return "Red";
    } else if (m_revBlinkin.get() == LEDConstants.kYellow) {
      return "Yellow";
    } else if (m_revBlinkin.get() == LEDConstants.kBlue) {
      return "Blue";
    } else if (m_revBlinkin.get() == LEDConstants.kWhite) {
      return "White";
    } else if (m_revBlinkin.get() == LEDConstants.kRainbowLavaPalette) {
      return "Rainbow (Lava)";
    } else if (m_revBlinkin.get() == LEDConstants.kSinelonLavaPalette) {
      return "Sinelon (Lava)";
    } else if (m_revBlinkin.get() == LEDConstants.kBeatsPerMinuteLavaPalette) {
      return "Beats Per Minute (Lava)";
    } else if (m_revBlinkin.get() == LEDConstants.kFireMedium) {
      return "Fire (Medium)";
    } else if (m_revBlinkin.get() == LEDConstants.kFireLarge) {
      return "Fire (Large)";
    } else if (m_revBlinkin.get() == LEDConstants.kTwinklesLavaPalette) {
      return "Twinkles (Lava)";
    } else if (m_revBlinkin.get() == LEDConstants.kColorWavesLavaPalette) {
      return "Color Waves (Lava)";
    } else if (m_revBlinkin.get() == LEDConstants.kLarsonScannerRed) {
      return "Larson Scanner (Red)";
    } else if (m_revBlinkin.get() == LEDConstants.kLightChaseRed) {
      return "Light Chase (Red)";
    } else if (m_revBlinkin.get() == LEDConstants.kHeartbeatRed) {
      return "Heartbeat (Red)";
    } else if (m_revBlinkin.get() == LEDConstants.kBreatheRed) {
      return "Breathe (Red)";
    } else if (m_revBlinkin.get() == LEDConstants.kStrobeRed) {
      return "Strobe (Red)";
    } else if (m_revBlinkin.get() == LEDConstants.kRainbow) {
      return "RAINBOW";
    } else {
      return "Color Not Detected";
    }
  }

  /**
   * Cycles through the colors in the m_colors array
   */
  public void cycleColor() {
    m_colorSelection += 1;
    if (m_colorSelection >= m_colors.length) {
      m_colorSelection = 0;
    }
    if (m_colorSelection < 0) {
      m_colorSelection = m_colors.length - 1;
    }

    setColor(m_colors[m_colorSelection]);
  }

  public void updateDashboard() {
    m_autoTab.addString("LED Color", new Supplier<String>(){
      @Override
      public String get() {
        return getCurrentColorString();
      }
    });
  }

  @Override
  public void periodic() {
    updateDashboard();
  }
}
