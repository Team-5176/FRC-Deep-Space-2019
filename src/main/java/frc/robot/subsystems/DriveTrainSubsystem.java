/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.RobotMap;
import frc.robot.commands.DriveTrainMove;

/**
 * Add your docs here.
 */
public class DriveTrainSubsystem extends Subsystem {
  public static VictorSP frontLeftMotor = new VictorSP(RobotMap.frontLeftMotorPort);
  public static VictorSP frontRightMotor = new VictorSP(RobotMap.frontRightMotorPort);
  public static VictorSP backLeftMotor = new VictorSP(RobotMap.backLeftMotorPort);
  public static VictorSP backRightMotor = new VictorSP(RobotMap.backRightMotorPort);

  public static MecanumDrive mecanumDrive =
    new MecanumDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public void moveMecanumDrive(double joyX, double joyY, double joyZ) { // tradition has been broken :(
    /* also its the function to drive the robot */
    if (joyX < 0.1 && joyX > -0.1) {
      // eliminate small joystick wobble
      joyX = 0;
    }
    if (joyY < 0.1 && joyY > -0.1) {
      // eliminate small joystick wobble
      joyY = 0;
    }

    if (joyX > RobotMap.MAX_DRIVE_SPEED) {
      joyX = RobotMap.MAX_DRIVE_SPEED;
    }

    if (joyX < -RobotMap.MAX_DRIVE_SPEED) {
      joyX = -RobotMap.MAX_DRIVE_SPEED;
    }

    if (joyY > RobotMap.MAX_DRIVE_SPEED) {
      joyY = RobotMap.MAX_DRIVE_SPEED;
    }

    if (joyY < -RobotMap.MAX_DRIVE_SPEED) {
      joyY = -RobotMap.MAX_DRIVE_SPEED;
    }

    mecanumDrive.driveCartesian(joyY, joyX, joyZ);
    // mecanumDrive.driveCartesian(1.0, 1.0, 1.0); // only for the memes :D
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DriveTrainMove());
  }
}
