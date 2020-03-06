/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.*;
import frc.robot.commands.Autonomous;
import frc.robot.commands.LiftElevator;
import frc.robot.commands.PID_ShooterCommand;
import frc.robot.commands.SpinRotations;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

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
        // private final OrchestraSystem m_orchestra = new OrchestraSystem(m_drivetrain); // MEMES

        // Define all controllers
        // driver: DT & related functions
        private final XboxController m_driverController = new XboxController(OIConstants.driverXboxPort);
        // operator: all other mechanisms (currently not sure if necessary)
        // private final XboxController m_operatorController = new XboxController(OIConstants.operatorXboxPort);

        // Create gyro object
        private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

        // Array of Autonomous Paths (from PathWeaver)
        private Trajectory[] m_paths = new Trajectory[] { 
                PathWeaver.getTrajectory("ENEMY_TRENCH_MOVE"), // index 0
                PathWeaver.getTrajectory("ENEMY_TRENCH_SHOOT"), // index 1
                PathWeaver.getTrajectory("ALLY_TRENCH_MOVE"), // index 2
                PathWeaver.getTrajectory("ALLY_TRENCH_SHOOT"), // index 3
                PathWeaver.getTrajectory("CENTER_AREA_MOVE"), // index 4
                PathWeaver.getTrajectory("CENTER_AREA_SHOOT") // index 5
        };

        // Create Command for Autonomous
        private final Autonomous m_allyTrench = new Autonomous(m_drivetrain, m_intake, m_shooter, m_visionSystem, m_gyro, m_hopper, 
                                                        m_paths[0], 
                                                        m_paths[1], 
                                                        m_paths[2], 
                                                        m_paths[3]
                                                        );
        private final Autonomous m_centerArea = new Autonomous(m_drivetrain, m_intake, m_shooter, m_visionSystem, m_gyro, m_hopper, 
                                                        m_paths[0], 
                                                        m_paths[1], 
                                                        m_paths[4], 
                                                        m_paths[5]
                                                        );

        // Create sendable choosers
        SendableChooser<Command> m_autoChooser = new SendableChooser<>();
        SendableChooser<Color> m_colorChooser = new SendableChooser<>();

        // Create Shuffleboard Tabs
        private final ShuffleboardTab m_autoTab = Shuffleboard.getTab("Auto");
        private final ShuffleboardTab m_teleopTab = Shuffleboard.getTab("Teleop");

        // Compressor
        private Compressor m_compressor = new Compressor(0);

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
                                () -> m_drivetrain.curvatureDrive(
                                                -m_driverController.getY(Hand.kLeft) * m_speedy,
                                                m_driverController.getX(Hand.kRight) * m_speedy,
                                                m_driverController.getBumper(Hand.kRight)),
                                m_drivetrain));

                // AUTONOMOUS CHOOSER
                m_autoChooser.setDefaultOption("Enemy Trench", m_allyTrench);
                m_autoChooser.addOption("Center Area", m_centerArea);
                m_autoTab.add(m_autoChooser);

                // COLOR CHOOSER
                m_colorChooser.setDefaultOption("Red", ColorWheelConstants.kRedTarget);
                m_colorChooser.addOption("Green", ColorWheelConstants.kGreenTarget);
                m_colorChooser.addOption("Blue", ColorWheelConstants.kBlueTarget);
                m_colorChooser.addOption("Yellow", ColorWheelConstants.kYellowTarget);
                m_teleopTab.add(m_colorChooser);
                
                // COMPRESSOR: true turns on compressor (auto stop @ 120psi) & false turns off compressor
                m_compressor.setClosedLoopControl(true); // not recommended to mess with this
        }

        public AHRS getGyro() {
                return m_gyro;
        }

        public VisionSystem getVisionSystem() {
                return m_visionSystem;
        }

        public Compressor getCompressor() {
                return m_compressor;
        }

        private void configureButtonBindings() {
                /*
                 * We setup inline commands using Runnables instead of command files- This is
                 * because it's just overly verbose to create an entire file for it The only
                 * time we create command files is for elaborate autonomous programs
                 */

                // DRIVETRAIN (live controls)
                new POVButton(m_driverController, OIConstants.b_moreSpeedy).whenPressed(() -> m_speedy += 0.05);
                new POVButton(m_driverController, OIConstants.b_lessSpeedy).whenPressed(() -> m_speedy -= 0.05);
                new JoystickButton(m_driverController, OIConstants.b_swappy.value).whenPressed(() -> {
                        m_drivetrain.setDrivetrainInverted(!m_swappy);
                        m_swappy = !m_swappy;
                }, m_drivetrain);

                // CLIMB SYSTEM
                new JoystickButton(m_driverController, OIConstants.b_elevatorLow.value)
                        .whenPressed(new LiftElevator(m_climb, ClimbConstants.kElevatorLowPosition));
                new JoystickButton(m_driverController, OIConstants.b_elevatorHigh.value)
                        .whenPressed(new LiftElevator(m_climb, ClimbConstants.kElevatorHighPosition));
                new JoystickButton(m_driverController, OIConstants.b_winchUp.value)
                        .whileHeld(() -> m_climb.setWinch(ClimbConstants.kWinchSpeed), m_climb)
                        .whenReleased(() -> m_climb.setWinch(0), m_climb);
                new JoystickButton(m_driverController, OIConstants.b_winchDown.value)
                        .whileHeld(() -> m_climb.setWinch(-ClimbConstants.kWinchSpeed), m_climb)
                        .whenReleased(() -> m_climb.setWinch(0), m_climb);

                // COLORWHEEL SYSTEM
                // new JoystickButton(m_driverController, OIConstants.b_colorWheel.value)
                //         .whileHeld(() -> m_colorSpinner.setColorSpinner(ColorWheelConstants.kColorSpinnerSpeed), m_colorSpinner)
                //         .whenReleased(() -> m_colorSpinner.setColorSpinner(0), m_colorSpinner);
                new JoystickButton(m_driverController, OIConstants.b_colorWheel.value)
                        .whenPressed(new SpinRotations(m_colorSpinner, 3));
                new JoystickButton(m_driverController, OIConstants.b_colorWheel.value)
                        .whileHeld(() -> m_colorSpinner.spinToColor(m_colorChooser.getSelected()), m_colorSpinner);

                // INTAKE SYSTEM
                new JoystickButton(m_driverController, OIConstants.b_intake.value)
                        .whileHeld(() -> {
                                m_intake.setIntake(IntakeConstants.kIntakeSpeed);
                                m_hopper.setHopper(HopperConstants.kHopperSpeed);
                        }, m_intake, m_hopper)
                        .whenReleased(() -> {
                                m_intake.setIntake(0);
                                m_hopper.setHopper(0);
                        }, m_intake, m_hopper);
                // new TriggerButton(m_driverController, OIConstants.b_intake.value)
                //         .whileActiveContinuous(() -> {
                //                 m_intake.setIntake(IntakeConstants.kIntakeSpeed);
                //                 m_hopper.setHopper(HopperConstants.kHopperSpeed);
                //         }, m_intake, m_hopper)
                //         .whenInactive(() -> {
                //                 m_intake.setIntake(0);
                //                 m_hopper.setHopper(0);
                //         }, m_intake, m_hopper);

                // HOPPER SYSTEM
                new JoystickButton(m_driverController, OIConstants.b_hopper.value)
                        .whenPressed(() -> m_hopper.setSolenoid(true), m_hopper)
                        .whileHeld(() -> m_hopper.setHopper(HopperConstants.kHopperSpeed), m_hopper)
                        .whenReleased(() -> {
                                m_hopper.setHopper(0);
                                m_hopper.setSolenoid(false);
                        }, m_hopper);
                new JoystickButton(m_driverController, OIConstants.b_hopperPiston.value)
                        .whileHeld(() -> m_hopper.toggleSolenoid(), m_hopper);

                // SHOOTER SYSTEM
                new JoystickButton(m_driverController, OIConstants.b_shooterPID.value)
                        .whenPressed(new PID_ShooterCommand(m_shooter, 48), true);
                new JoystickButton(m_driverController, OIConstants.b_shooter.value)
                        .whileHeld(new InstantCommand(()->m_shooter.manualSpinMotor(0.2), m_shooter));

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
                // new JoystickButton(m_driverController, OIConstants.b_togglePauseMusic.value)
                //         .whenPressed(() -> m_orchestra.togglePauseMusic(), m_orchestra);
                // new JoystickButton(m_driverController, OIConstants.b_toggleStopMusic.value)
                //         .whenPressed(() -> m_orchestra.toggleStopMusic(), m_orchestra);
                // new JoystickButton(m_driverController, OIConstants.b_nextSong.value)
                //         .whenPressed(() -> m_orchestra.nextSong(), m_orchestra);
                // new JoystickButton(m_driverController, OIConstants.b_prevSong.value)
                //         .whenPressed(() -> m_orchestra.previousSong(), m_orchestra);
        }

        // Autonomous
        public Command getAutonomousCommand() {
                // Chosen command will run
                return m_autoChooser.getSelected();
        }
}