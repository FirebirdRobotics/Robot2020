/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    // OI
    public static final class OIConstants {
        // CONTROLLERS
        public static final int driverXboxPort = 0;
        public static final int operatorXboxPort = 1; // unsure if this will be used as of now

        // CLIMB SYSTEM
        public static final Button b_elevatorLow = Button.kA;
        public static final Button b_elevatorHigh = Button.kA;
        public static final Button b_skiLiftLeft = Button.kA;
        public static final Button b_skiLiftRight = Button.kA;

        // COLORWHEEL SYSTEM
        public static final Button b_colorWheel = Button.kA;

        // HOPPER SYSTEM
        public static final Button b_hopper = Button.kA;
        public static final Button b_hopperPiston = Button.kA;

        // INTAKE SYSTEM
        public static final Button b_intake = Button.kA;

        // SHOOTER SYSTEM
        public static final Button b_shooter = Button.kA;
        public static final Button b_shooterPID = Button.kA;

        // VISIONSYSTEM
        public static final Button b_visionRoutineTape = Button.kA;
    }

    // Autonomous
    public static final class AutonomousConstants {
        // Drivetrain in auto
        public static final double kDriveSpeed = 0.5;
        public static final double kTurnSpeed = 0.5;
        public static final double kWheelRadius = 2.5; // inches
        public static final double kRobotRadius = 0;

        // Time constants
        public static final double kWaitTime = 2.0; // seconds, time to wait for auto centering (limelight)
        public static final double kShooterWarmUpTime = 0.5; // seconds, time to wait for shooter to warm up
        public static final double kHopperWaitTime = 2.0; // seconds, time for one ball to pass from hopper to shooter        
        public static final double kShooterWaitTime = 2.0; // seconds, time for one ball to shoot        
    }

    // Drivetrain
    public static final class DriveConstants {
        // Speed constants
        public static final double kDriveSpeed = 0.50;
        public static final double kTurnSpeed = 0.50;

        // TalonFX CAN ports
        public static final int dtFrontLeftPort = 1;
        public static final int dtBackLeftPort = 2;
        public static final int dtFrontRightPort = 3;
        public static final int dtBackRightPort = 4;

        // Current limiting constants
        public static final boolean kCurrentLimitingEnabled = true; // True/False to enable/disable limit feature.
        public static final int kPeakCurrentAmps = 15; // The "holding" current (amperes) to limit to when feature is activated.
        public static final int kPeakTimeMs = 0; // How long current must exceed threshold (seconds) before limiting occurs.
        public static final int kContCurrentAmps = 10; // Current must exceed this threshold (amperes) before limiting occurs.
        public static final int kTimeoutMs = 30; // Amount of time (in milliseconds) to wait for a specified element to be found before an error is thrown

        // Deadband: makes sure controllers dont move motors if joystick is accidentally moved a small amount
        public static final double kDeadband = 0.05;
    }

    // Climbing
    public static final class ClimbConstants {
        // Speed constants
        public static final double kSkiLiftSpeed = 0.50; // constant speed for ski lift
        public static final double kElevatorSpeed = 0.50; // speed that elevator rises

        // SparkMAX CAN ports
        public static final int skiLiftPort = 5;
        public static final int elevatorPort = 6;

        // Encoder counts
        public static final double kElevatorLowPosition = 0;
        public static final double kElevatorHighPosition = 5000;
    }

    // Color wheel
    public static final class ColorWheelConstants {
        // Speed constants
        public static final double kColorSpinnerSpeed = 0.50;

        // SparkMAX CAN port
        public static final int colorWheelPort = 7;
        
        // Solenoid port
        public static final int spinnerSolenoid = 0;

        // Color targets
        public static final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
        public static final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
        public static final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
        public static final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
        // 
    }

    // Intake
    public static final class IntakeConstants {
        // Speed constants
        public static final double kIntakeSpeed = 0.50;

        // SparkMAX CAN ports
        public static final int intakePort = 8;

        // Solenoid ports
        public static final int intakeSolenoidRight = 1;
        public static final int intakeSolenoidLeft = 2;
    }

    // Hopper
    public static final class HopperConstants {
        // Speed constants
        public static final double kHopperSpeed = 0.50;

        // SparkMAX CAN ports
        public static final int hopperPort = 9;

        // Solenoid ports
        public static final int hopperSolenoid = 3;
    }

    // Shooter
    public static final class ShooterConstants {
        // Target RPM for shooter
        public static final double motorRPM = 5000;
        
        // SparkMAX CAN ports
        public static final int shooterFirstPort = 10;
        public static final int shooterSecondPort = 11;

        // PID Constants
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;

        // Solenoid port
        public static final int shooterHoodSolenoid = 4;

        // Other constants
        public static final double motorRadius = 0 * UnitConversionConstants.distanceConversionFactor;
        public static final double shooterAngle = 0 * UnitConversionConstants.angleConversionFactor;
        public static final double shooterHeight = 0 * UnitConversionConstants.distanceConversionFactor;
    }

    // LEDs
    public static final class LEDConstants {
        public static final int revBlinkinPort = 12;

        // Rev Blinkin, page 14: http://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf
        public static final double kRed = 0.61;
        public static final double kYellow = 0.69;
        public static final double kBlack = 0.99;
    }

    // Vision
    public static final class VisionConstants {
        // PID Constants (turn)
        public static final double kpRotation = 0.20; // proportional

        // PID Constants (drive)
        public static double kpDistance = 0.70; // proportional
        public static double kiDistance = 0.00; // integral
        public static double kdDistance = 0.00; // derivative
        public static final double kTimePerLoop = 0.02; // robot updates every 0.02 seconds

        // Minimum power given to DT needed to overcome friction; helps robot move & rotate at low speeds
        public static double kConstantForce = 0.05;
        
        // Maximum power (speed constant) to keep drivetrain at
        public static final double kMaxDrive = 0.60;
        public static final double kMaxTurn = 0.80;

        // CLOSEST TARGET STUFF
        public static final double[] kTargetAreas = { 6.0, 2.5, 0.85 }; // INPUT YOUR WANTED TARGETAREA VALUES HERE (% of screen)
        public static final double[] kTargetDistances = { 20, 40, 60 }; // INPUT YOUR WANTED DISTANCES HERE (assume inches)
        public static double[] distancesToTargets = new double[kTargetAreas.length]; // change this to the length of whichever array you are using
        public static double closestTargetArea = 0; // for using target area
        public static double closestTargetDistance = 0; // for using target distance

        // DISTANCE CALC STUFF (trig)
        public static final double kTargetHeight = FieldConstants.kTargetHeight; // height of target above floor (in)
        public static final double kLimelightHeight = 47.5; // height of camera above floor (in)
        public static final double kMountingAngle = 30.0; // angle that camera is mounted at (deg)
    }

    // Unit Conversion because the Math java functions use specific units
    public static class UnitConversionConstants {
        public static final double angleConversionFactor = (Math.PI / 180); // angles to radians
        public static final double distanceConversionFactor = 39.37; // inches to meters
    }

    public static final class PhysicsConstants {
        public static final double gAcceleration = 386.09; // in inches per second squared
    }

    public static final class MotorConstants {
        public static final double kNeoRPM = 5880; // max RPM of NEOs
        public static final double kFalconRPM = 6380; // max RPM of Falcon500s
    }

    public static final class FieldConstants {
        // All measurements are in inches
        public static final double kInitiationLine = 120.0;
        public static final double kTrenchToDriverStation = 200.0;
        public static final double kInitiationLineToTrench = 80.0;
        public static final double kTargetHeight = 98.25;
        public static final double kOuterToInnerTarget = 29.25;
    }
}