/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class DriveTrainMove extends Command {
  public DriveTrainMove() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.literallyTheDriveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    shouldIDrive = true;
    slowMultiplier = 1;
  }

  // for slow button
  double slowMultiplier = 1;

  // for vision toggle button
  boolean lastButtonPress2 = false;
  boolean state2 = false;
  int differenceCounter2 = 0;

  public static boolean shouldIDrive = true;

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (Robot.oi.pilotJoystick.getRawButton(RobotMap.LOGITECH_LB)) {
      slowMultiplier = 0.5;
    } else {
      slowMultiplier = 1;
    }

    double pilotRawAxis0 = Robot.oi.pilotJoystick.getRawAxis(0);
    // the meat boi
    // double joyX = -Robot.oi.pilotJoystick.getX() * 0.8;
    double joyX = -pilotRawAxis0 * 0.8; // old x axis; left stick left to right
    // double joyY = Robot.oi.pilotJoystick.getY() * 0.5;
    double joyY = Robot.oi.pilotJoystick.getRawAxis(1) * 0.5; // old y axis; left stick up to down
    // double joyZ = -Robot.oi.pilotJoystick.getZ() * 0.5;
    double joyZ = -Robot.oi.pilotJoystick.getRawAxis(4) * 0.5; // old z axis; right stick left to right

    //SmartDashboard.putNumber("joyX", joyX);
    //SmartDashboard.putNumber("joyZ", joyZ);

    if (pilotRawAxis0 > 0.5) {
      joyZ = RobotMap.MECANUM_FIX_RIGHT_THING;
    }

    if (shouldIDrive) {
      Robot.literallyTheDriveTrain.moveMecanumDrive(joyY * slowMultiplier, joyX * slowMultiplier, joyZ * slowMultiplier);
    }
    // DriverStation.reportWarning("moving: " + joyY + " " + joyX + " " + joyZ, false);

    double limeX = RobotMap.limelightTx.getDouble(0.0);
    // double limeY = RobotMap.limelightTy.getDouble(0.0);
    // double limeA = RobotMap.limelightTa.getDouble(0.0);
    boolean limeHasTarget = RobotMap.limelightTv.getDouble(0.0) == 1;

    // boolean currentPress2 = Robot.oi.pilotJoystick.getRawButton(RobotMap.VISION_TOGGLE_BUTTON);
    // boolean currentPress2 = Robot.oi.coPilotJoystick.getRawButton(RobotMap.LOGITECH_Y);
    boolean currentPress2 = Robot.oi.pilotJoystick.getRawButton(RobotMap.LOGITECH_X);
    boolean isDifferenceBetweenPresses2 = !(currentPress2 == lastButtonPress2);

    if (isDifferenceBetweenPresses2) {
      differenceCounter2++;
      if (differenceCounter2 >= 2) {
        differenceCounter2 = 0;
        state2 = !state2;
      }
    }
    if (state2) {
      // we will do the vision stuff
      // DriverStation.reportWarning("thing", false);
      SmartDashboard.putBoolean("VisionOn", true);
      if (limeHasTarget) {
        // if (limeX < RobotMap.VISION_RANGE && limeX > -RobotMap.VISION_RANGE) {
        //   limeX = 0.0;
        // }
        if (limeX > RobotMap.VISION_RANGE) {
          // start aligning the robot to the right
          // DriverStation.reportWarning("trying to move right with " + limeX, false);
          // Robot.literallyTheDriveTrain.moveMecanumDrive(0.0, 0.0, -RobotMap.VISION_AUTO_SPEED);
          Robot.literallyTheDriveTrain.moveMecanumDrive(0.0, -RobotMap.VISION_AUTO_SPEED, 0.0);
        } else if (limeX < -RobotMap.VISION_RANGE) {
          // align robot to the left
          // DriverStation.reportWarning("trying to move left with " + limeX, false);
          // Robot.literallyTheDriveTrain.moveMecanumDrive(0.0, 0.0, RobotMap.VISION_AUTO_SPEED);
          Robot.literallyTheDriveTrain.moveMecanumDrive(0.0, RobotMap.VISION_AUTO_SPEED, 0.0);
        } else if (limeX > RobotMap.VISION_PRECISION_RANGE) {
          // start aligning the robot to the right precisely
          // DriverStation.reportWarning("trying to move right precisely with " + limeX, false);
          // Robot.literallyTheDriveTrain.moveMecanumDrive(0.0, 0.0, -RobotMap.VISION_PRECISION_SPEED);
          Robot.literallyTheDriveTrain.moveMecanumDrive(0.0, -RobotMap.VISION_PRECISION_SPEED, 0.0);
        } else if (limeX < -RobotMap.VISION_PRECISION_RANGE) {
          // start aligning the robot to the left precisely
          // DriverStation.reportWarning("trying to move left precisely with " + limeX, false);
          // Robot.literallyTheDriveTrain.moveMecanumDrive(0.0, 0.0, RobotMap.VISION_PRECISION_SPEED);
          Robot.literallyTheDriveTrain.moveMecanumDrive(0.0, RobotMap.VISION_PRECISION_SPEED, 0.0);
        }
      }
    } else {
      // not doing vision stuff
      SmartDashboard.putBoolean("VisionOn", false);
    }
    lastButtonPress2 = currentPress2;

    // SmartDashboard.putNumber("ult", RobotMap.ultrasonicBoi.getVoltage());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
