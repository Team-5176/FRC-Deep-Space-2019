/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Ultrasonic;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // Limelight Stuff
  public static NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  public static NetworkTableEntry limelightTx = limelightTable.getEntry("tx");
  public static NetworkTableEntry limelightTy = limelightTable.getEntry("ty");
  public static NetworkTableEntry limelightTa = limelightTable.getEntry("ta");
  public static NetworkTableEntry limelightTv = limelightTable.getEntry("tv");

  public static Compressor literallyTheCompressor = new Compressor(0);

  public static AnalogInput ultrasonicBoi = new AnalogInput(0);
 
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;

  // PWM
  public static int testMotorPort = 0; // we are putting a vacuum pump on this port
  public static int secondVacuumPort = 8;
  public static int thirdVacuumPort = 9;
  public static int testMotorPort2 = 5;

  public static int frontLeftMotorPort = 1;
  public static int frontRightMotorPort = 2;
  public static int backLeftMotorPort = 3;
  public static int backRightMotorPort = 4;

  public static int frontLeftCANMotorPort = 2;
  public static int frontRightCANMotorPort = 3;
  public static int backLeftCANMotorPort = 4;
  public static int backRightCANMotorPort = 5;

  public static int frontClimberMotorPort = 6;
  public static int rearClimberMotorPort = 7;

  // DIO
  public static int climberFrontLimitSwitchPort = 0;
  public static int climberRearLimitSwitchPort = 9;

  // CAN

  // USB
  public static int joystickPort = 0;
  public static int coPilotJoystickPort = 1;

  public static int camera0Port = 0;
  public static int camera1Port = 1;

  // LOGITECH CONTROLLER MAP: (this is old and not accurate)
  // 1:X
  // 2:A
  // 3:B
  // 4:Y
  // NEW ACCURATE LOGITECH MAP (set to X)
  public static final int LOGITECH_A = 1;
  public static final int LOGITECH_B = 2;
  public static final int LOGITECH_X = 3;
  public static final int LOGITECH_Y = 4;
  
  // Begin pilotJoystick mappings
  public static final int VACUUM_TOGGLE_BUTTON = LOGITECH_A; // A
  public static final int VISION_TOGGLE_BUTTON = LOGITECH_X; // X
  public static final int VACUUM_SOLENOID_BUTTON = LOGITECH_Y; // Y
  public static final int PNEUMATIC_ARM_TOGGLE_BUTTON = LOGITECH_B; // B
  // End pilotJoystick mappings

  // Begin coPilotJoystick mappings
  public static int manualRearClimbOverride = LOGITECH_X; // X
  public static int manualFrontClimbOverride = LOGITECH_A; // A
  // End coPilotJoystick mappings

  // Other Drivetrain Constants
  public static double maxSpeed = 1;
  public static final double MAX_DRIVE_SPEED =1;// 0.6;
  public static double vacuumSpeed = 0.2;
  public static final double VISION_RANGE = 3.0;
  public static final double VISION_AUTO_SPEED = 0.3;
  public static final double VISION_PRECISION_RANGE = 1.0;
  public static final double VISION_PRECISION_SPEED = 0.2;

  // public static final double PANIC_CLIMB_SPEED_FRONT = 0.15; // subject to change
  // public static final double PANIC_CLIMB_SPEED_REAR = 0.08; // subject to change
  // // public static final double GO_CLIMB_SPEED_FRONT = 0.50;
  // // public static final double GO_CLIMB_SPEED_REAR = 0.455;
  // public static final double GO_CLIMB_SPEED_FRONT = 0.35;
  // public static final double GO_CLIMB_SPEED_REAR = 0.30;

  // for testing
  public static final double PANIC_CLIMB_SPEED_FRONT = 0.15; // subject to change
  public static final double PANIC_CLIMB_SPEED_REAR = 0.08; // subject to change
  // public static final double GO_CLIMB_SPEED_FRONT = 0.50;
  // public static final double GO_CLIMB_SPEED_REAR = 0.455;
  public static final double GO_CLIMB_SPEED_FRONT = 0.25;
  public static final double GO_CLIMB_SPEED_REAR = 0.20;

  // PCM
  public static final int VACUUM_SOLENOID_0_A = 7;
  public static final int VACUUM_SOLENOID_0_B = 0;
  public static final int VACUUM_SOLENOID_1_A = 6;
  public static final int VACUUM_SOLENOID_1_B = 1;
  public static final int VACUUM_SOLENOID_2_A = 5;
  public static final int VACUUM_SOLENOID_2_B = 2;
  public static final int ARM_SOLENOID_0_A = 4;
  public static final int ARM_SOLENOID_0_B = 3;

  // The following is the constants to modify the custom MecanumDrive class
  public static final double MECANUM_FRONT_LEFT_MULTIPLIER = 1.0;
  public static final double MECANUM_FRONT_RIGHT_MULTIPLIER = 1.0;
  public static final double MECANUM_REAR_LEFT_MULTIPLIER = 1.0;
  public static final double MECANUM_REAR_RIGHT_MULTIPLIER = 0.75;

  public static final double MECANUM_FIX_RIGHT_THING = -0.04;
}
