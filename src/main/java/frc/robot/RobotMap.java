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

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  public static NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  public static NetworkTableEntry limelightTx = limelightTable.getEntry("tx");
  public static NetworkTableEntry limelightTy = limelightTable.getEntry("ty");
  public static NetworkTableEntry limelightTa = limelightTable.getEntry("ta");
  public static NetworkTableEntry limelightTv = limelightTable.getEntry("tv");
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;

  // PWM
  public static int testMotorPort = 0;
  public static int testMotorPort2 = 5;

  public static int frontLeftMotorPort = 1;
  public static int frontRightMotorPort = 2;
  public static int backLeftMotorPort = 3;
  public static int backRightMotorPort = 4;

  // DIO

  // CAN

  // USB
  public static int joystickPort = 0;
  public static int coPilotJoystickPort = 1;

  public static int camera0Port = 0;
  public static int camera1Port = 1;

  // Joystick port 2 is A on the logitechs
  public static int motorToggleButton = 2;
  // Joystick port 1 is X on the logitechs
  public static int secondToggleButton = 1;
  // Joystick port 4 is Y on the logitechs
  public static int pneumaticToggleButton = 4;
  // Joystick port 3 is B on the logitechs
  public static int pneumaticToggleButton2 = 3;

  // Other Drivetrain Constants
  public static double maxSpeed = 1;
  public static final double MAX_DRIVE_SPEED =1;// 0.6;
  public static double vacuumSpeed = 0.2;
  public static final double VISION_RANGE = 3.0;
  public static final double VISION_AUTO_SPEED = 0.3;
  public static final double VISION_PRECISION_RANGE = 1.0;
  public static final double VISION_PRECISION_SPEED = 0.2;
}
