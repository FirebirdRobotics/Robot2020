/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This subsystem is literally just for memes;
 * The music is unable to play while the motors are running 
 * Music goes into the deploy folder (as .chrp files)
 */
public class OrchestraSystem extends SubsystemBase {
 
  private Orchestra m_orchestra;

  private int m_songSelection;
  private int m_loadingTimeLoops = 10;

  // ADD CHRP FILE NAMES HERE -- actual chirp files go in deploy
  private String[] m_songs = new String[] {
    "song1.chrp",
    "song2.chrp" // etc.
  };

  private ArrayList<TalonFX> m_instruments = new ArrayList<TalonFX>();

  public OrchestraSystem(Drivetrain drivetrain) {

    WPI_TalonFX[] talons = drivetrain.getMotors();

    /* Initialize the TalonFX's to be used */
    for (int i = 0; i < talons.length; i++) {
      m_instruments.add(talons[i]);
    }

    m_orchestra = new Orchestra(m_instruments);
  }

  public void loadMusicSelection(int offset) {
    /* increment song selection */
    m_songSelection += offset;
    /* wrap song index in case it exceeds boundary */
    if (m_songSelection >= m_songs.length) {
        m_songSelection = 0;
    }
    if (m_songSelection < 0) {
        m_songSelection = m_songs.length - 1;
    }
    /* load the chirp file */
    m_orchestra.loadMusic(m_songs[m_songSelection]); 

    /* print to console */
    System.out.println("Current song: " + m_songs[m_songSelection]);
    
    /* schedule a play request, after a delay.  
        This gives the Orchestra service time to parse chirp file.
        If play() is called immedietely after, you may get an invalid action error code. */
    m_loadingTimeLoops = 10;
  }

  public void togglePauseMusic() {
    if (m_orchestra.isPlaying()) {
      m_orchestra.pause();
      System.out.println("Song paused.");
    }  else {
      m_orchestra.play();
      System.out.println("Playing song...");
    }
  }

  public void toggleStopMusic() {
    if (m_orchestra.isPlaying()) {
      m_orchestra.stop();
      System.out.println("Song stopped.");
    }  else {
      m_orchestra.play();
      System.out.println("Playing song...");
    }
  }

  public void nextSong() {
    loadMusicSelection(+1);
  }

  public void previousSong() {
    loadMusicSelection(-1);
  }

  @Override
  public void periodic() {
    /* if song selection changed, auto-play it */
    if (m_loadingTimeLoops > 0) {
      m_loadingTimeLoops--;
      if (m_loadingTimeLoops == 0) {
          /* scheduled play request */
          System.out.println("Auto-playing song.");
          m_orchestra.play();
      }
    }
  }  
}
 