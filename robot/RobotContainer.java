/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.*;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

    // Define all Commands
    // private final ExampleCommand m_command = new ExampleCommand();

    // Define all controllers
    private final XboxController m_driverController = new XboxController(OIConstants.driverXboxPort); // driver: DT &
                                                                                                      // related
                                                                                                      // functions
    private final XboxController m_operatorController = new XboxController(OIConstants.operatorXboxPort); // operator:
                                                                                                          // all other
                                                                                                          // mechanisms

    // Create a sendable chooser for auto programs
    // SendableChooser<Command> m_chooser = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings w/ the below function
        configureButtonBindings();

        // Configure default commands
        m_drivetrain
                .setDefaultCommand(
                        new RunCommand(
                                () -> m_drivetrain.curvatureDrive(m_driverController.getY(Hand.kLeft),
                                        m_driverController.getX(Hand.kLeft), m_driverController.getXButton()),
                                m_drivetrain));
    }

  private void configureButtonBindings() {
    /*
     * We setup inline commands using Runnables instead of command files- This is
     * because it's just overly verbose to create an entire file for it The only
     * time we create command files is for elaborate autonomous programs
     */

    // CLIMB SYSTEM
    new JoystickButton(m_driverController, OIConstants.b_elevatorLow.value)
        .whenPressed(() -> m_climb.setElevatorPosition(ClimbConstants.kElevatorLowPosition), m_climb);
    new JoystickButton(m_driverController, OIConstants.b_elevatorHigh.value)
        .whenPressed(() -> m_climb.setElevatorPosition(ClimbConstants.kElevatorHighPosition), m_climb);
    new JoystickButton(m_driverController, OIConstants.b_skiLiftRight.value)
        .whileHeld(() -> m_climb.moveSkiLift(ClimbConstants.kSkiLiftSpeed), m_climb);
    new JoystickButton(m_driverController, OIConstants.b_skiLiftLeft.value)
        .whileHeld(() -> m_climb.moveSkiLift(-ClimbConstants.kSkiLiftSpeed), m_climb);

    // COLORWHEEL SYSTEM
    new JoystickButton(m_driverController, OIConstants.b_colorWheel.value)
        .whileHeld(() -> m_colorSpinner.spinColorWheel(ColorWheelConstants.kColorSpinnerSpeed), m_colorSpinner);

    // HOPPER SYSTEM
    new JoystickButton(m_driverController, OIConstants.b_hopper.value)
        .whileHeld(() -> m_hopper.runHopper(HopperConstants.kHopperSpeed), m_hopper);

    // INTAKE SYSTEM
    new JoystickButton(m_driverController, OIConstants.b_intake.value)
        .whileHeld(() -> m_intake.runIntake(IntakeConstants.kIntakeSpeed), m_intake);

    // SHOOTER SYSTEM
    new JoystickButton(m_driverController, OIConstants.b_shooter.value)
        .whileHeld(() -> m_shooter.spinShooter(ShooterConstants.motorRPM), m_shooter);
    new JoystickButton(m_driverController, OIConstants.b_shooterPID.value)
        .whileHeld(new ShooterCommand(m_shooter, m_visionSystem), false);

    // VISION SYSTEM
    new JoystickButton(m_driverController, OIConstants.b_visionRoutineTape.value)
        .whenPressed(() -> m_visionSystem.visionRoutineTape(m_drivetrain))
        .whenReleased(() -> m_visionSystem.visionRoutineReleased(m_drivetrain));
  }

    // Uncomment this when we wanna add auto
    // public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // return m_autoCommand;
    // }
}