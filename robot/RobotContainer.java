/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.Constants.*;
import frc.robot.commands.AutoShooterCommand;
import frc.robot.commands.Autonomous;
import frc.robot.commands.LiftElevator;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Define all Subsystems
    private final Drivetrain m_drivetrain = new Drivetrain();
    private final VisionSystem m_visionSystem = new VisionSystem();
    private final ShooterSystem m_shooter = new ShooterSystem();
    private final ClimbSystem m_climb = new ClimbSystem();
    private final ColorWheelSystem m_colorSpinner = new ColorWheelSystem();
    private final HopperSystem m_hopper = new HopperSystem();
    private final IntakeSystem m_intake = new IntakeSystem();
    private final LEDSystem m_ledSystem = new LEDSystem();
    private final OrchestraSystem m_orchestra = new OrchestraSystem(m_drivetrain);

    // Define all controllers
    // driver: DT & related functions
    private final XboxController m_driverController = new XboxController(OIConstants.driverXboxPort);
    // operator: all other mechanisms (currently not sure if necessary)
    // private final XboxController m_operatorController = new XboxController(OIConstants.operatorXboxPort);

    // Create gyro object
    private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

    // Array of Autonomous Paths (from PathWeaver)
    private Trajectory[] m_paths = new Trajectory[] { 
                PathWeaver.getTrajectory(""), // 1
                PathWeaver.getTrajectory(""), // 2
                PathWeaver.getTrajectory(""), // 3
                PathWeaver.getTrajectory(""), // 4
                PathWeaver.getTrajectory(""), // 5
                PathWeaver.getTrajectory(""), // 6
                PathWeaver.getTrajectory(""), // 7
                PathWeaver.getTrajectory(""), // 8
                PathWeaver.getTrajectory(""), // 9
                PathWeaver.getTrajectory(""), // 10
                PathWeaver.getTrajectory("")  // 11
        };
        
    // Create Command for Autonomous
    private final Autonomous m_enemyTrench = new Autonomous(m_drivetrain, m_intake, m_shooter, m_visionSystem, m_gyro, m_hopper,
                                                                m_paths[0],
                                                                m_paths[1],
                                                                m_paths[2],
                                                                m_paths[3]
                                                        );
    private final Autonomous m_middle = new Autonomous(m_drivetrain, m_intake, m_shooter, m_visionSystem, m_gyro, m_hopper,
                                                                m_paths[4],
                                                                m_paths[5],
                                                                m_paths[6],
                                                                m_paths[7]
                                                        );
    private final Autonomous m_allyTrench = new Autonomous(m_drivetrain, m_intake, m_shooter, m_visionSystem, m_gyro, m_hopper,
                                                                m_paths[8],
                                                                m_paths[9],
                                                                m_paths[10],
                                                                m_paths[11]
                                                        );

    // Create a sendable chooser for auto programs
    SendableChooser<Command> m_chooser = new SendableChooser<>();

    // Create Shuffleboard Tabs
    private ShuffleboardTab m_autoTab = Shuffleboard.getTab("Auto");

    // Variables for customizing the robot while it is live
    public double m_speedy = 0.2; // adds/subtracts speed from robot
    public boolean m_swappy = false; // tells whether to invert drivetrain

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings w/ the below function
        configureButtonBindings();

        // DRIVETRAIN
        m_drivetrain.setDefaultCommand(new RunCommand(
                () -> m_drivetrain.curvatureDrive(-m_driverController.getY(Hand.kLeft) * m_speedy,
                        m_driverController.getX(Hand.kRight) * m_speedy, m_driverController.getBumper(Hand.kRight)),
                m_drivetrain));

        // AUTONOMOUS
        m_chooser.setDefaultOption("Enemy Trench", m_enemyTrench);
        m_chooser.addOption("Middle", m_middle);
        m_chooser.addOption("Ally Trench", m_allyTrench);
        
        m_autoTab.add(m_chooser);
    }

    public AHRS getGyro() {
        return m_gyro;
    }

    public VisionSystem getVisionSystem() {
        return m_visionSystem;
    }

    private void configureButtonBindings() {
        /*
         * We setup inline commands using Runnables instead of command files- This is
         * because it's just overly verbose to create an entire file for it The only
         * time we create command files is for elaborate autonomous programs
         */

        // DRIVETRAIN (live controls)
        new JoystickButton(m_driverController, OIConstants.b_moreSpeedy.value).whenPressed(() -> m_speedy += 0.05);
        new JoystickButton(m_driverController, OIConstants.b_lessSpeedy.value).whenPressed(() -> m_speedy -= 0.05);
        new JoystickButton(m_driverController, OIConstants.b_swappy.value).whenPressed(() -> {  
                m_drivetrain.setDrivetrainInverted(!m_swappy);
                m_swappy = !m_swappy;
        }, m_drivetrain);

        // CLIMB SYSTEM
        new JoystickButton(m_driverController, OIConstants.b_elevatorLow.value)
                .whenPressed(new LiftElevator(m_climb, ClimbConstants.kElevatorLowPosition));
        new JoystickButton(m_driverController, OIConstants.b_elevatorHigh.value)
                .whenPressed(new LiftElevator(m_climb, ClimbConstants.kElevatorHighPosition));
        new JoystickButton(m_driverController, OIConstants.b_skiLiftRight.value)
                .whileHeld(() -> m_climb.moveSkiLift(ClimbConstants.kSkiLiftSpeed), m_climb);
        new JoystickButton(m_driverController, OIConstants.b_skiLiftLeft.value)
                .whileHeld(() -> m_climb.moveSkiLift(-ClimbConstants.kSkiLiftSpeed), m_climb);

        // COLORWHEEL SYSTEM
        new JoystickButton(m_driverController, OIConstants.b_colorWheel.value)
                .whileHeld(() -> m_colorSpinner.spinColorWheel(ColorWheelConstants.kColorSpinnerSpeed), m_colorSpinner);        

        // INTAKE SYSTEM
        new JoystickButton(m_driverController, OIConstants.b_intake.value)
                .whileHeld(() -> {
                        m_intake.setIntake(IntakeConstants.kIntakeSpeed);
                        m_hopper.setHopper(HopperConstants.kHopperSpeed);
                }, m_intake, m_hopper)
                .whenReleased(() -> {
                        m_intake.setIntake(0);
                        m_hopper.setHopper(0);
                });

        // HOPPER SYSTEM; note- if the advanced commands and stuff works, there's no need to have manual hopper
        // new JoystickButton(m_driverController, OIConstants.b_hopper.value)
        //         .whileHeld(() -> m_hopper.setHopper(HopperConstants.kHopperSpeed), m_hopper);
        // new JoystickButton(m_driverController, OIConstants.b_hopperPiston.value)
        //         .whileHeld(() -> m_hopper.toggleSolenoid(), m_hopper);

        // SHOOTER SYSTEM
        // new JoystickButton(m_driverController, OIConstants.b_shooter.value)
        //         .whileHeld(() -> m_shooter.spinShooter(ShooterConstants.motorRPM), m_shooter);
        new JoystickButton(m_driverController, OIConstants.b_shooterPID.value)
                .whenPressed(new AutoShooterCommand(m_shooter, m_hopper, m_visionSystem, 5), true);

        // VISION SYSTEM
        // new JoystickButton(m_driverController, OIConstants.b_visionRoutineTape.value)
        //         .whenPressed(() -> m_visionSystem.visionRoutineTape(m_drivetrain))
        //         .whenReleased(() -> m_visionSystem.visionRoutineReleased(m_drivetrain));
        new JoystickButton(m_driverController, OIConstants.b_visionRoutineTape.value)
                .whenPressed(() -> m_visionSystem.turnToTarget(m_drivetrain), m_drivetrain);

        // LED SYSTEM
        new JoystickButton(m_driverController, OIConstants.b_cycleLEDs.value)
                .whenPressed(() -> m_ledSystem.cycleColor(), m_ledSystem);

        // ORCHESTRA
        new JoystickButton(m_driverController, OIConstants.b_togglePauseMusic.value)
                .whenPressed(() -> m_orchestra.togglePauseMusic(), m_orchestra);
        new JoystickButton(m_driverController, OIConstants.b_toggleStopMusic.value)
                .whenPressed(() -> m_orchestra.toggleStopMusic(), m_orchestra);
        new JoystickButton(m_driverController, OIConstants.b_nextSong.value).whenPressed(() -> m_orchestra.nextSong(),
                m_orchestra);
        new JoystickButton(m_driverController, OIConstants.b_prevSong.value)
                .whenPressed(() -> m_orchestra.previousSong(), m_orchestra);
    }

    // Autonomous
    public Command getAutonomousCommand() {
        // Chosen command will run
        return m_chooser.getSelected();
    }
}