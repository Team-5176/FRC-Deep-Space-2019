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
  }

  // for vision toggle button
  boolean lastButtonPress2 = false;
  boolean state2 = false;
  int differenceCounter2 = 0;

  // Called repeatedly when this Command is scheduled to run
  @Override
    protected void execute() {
    // the meat boi
    double joyX = -Robot.oi.pilotJoystick.getX() * 0.8;
    double joyY = Robot.oi.pilotJoystick.getY() * 0.5;
    double joyZ = -Robot.oi.pilotJoystick.getZ() * 0.5;

    Robot.literallyTheDriveTrain.moveMecanumDrive(joyY, joyX, joyZ);
    // DriverStation.reportWarning("moving: " + joyY + " " + joyX + " " + joyZ, false);

    double limeX = RobotMap.limelightTx.getDouble(0.0);
    // double limeY = RobotMap.limelightTy.getDouble(0.0);
    // double limeA = RobotMap.limelightTa.getDouble(0.0);
    boolean limeHasTarget = RobotMap.limelightTv.getDouble(0.0) == 1;

    boolean currentPress2 = Robot.oi.pilotJoystick.getRawButton(RobotMap.secondToggleButton);
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
          DriverStation.reportWarning("trying to move right with " + limeX, false);
          Robot.literallyTheDriveTrain.moveMecanumDrive(0.0, 0.0, -RobotMap.VISION_AUTO_SPEED);
        } else if (limeX < -RobotMap.VISION_RANGE) {
          // align robot to the left
          DriverStation.reportWarning("trying to move left with " + limeX, false);
          Robot.literallyTheDriveTrain.moveMecanumDrive(0.0, 0.0, RobotMap.VISION_AUTO_SPEED);
        } else if (limeX > RobotMap.VISION_PRECISION_RANGE) {
          // start aligning the robot to the right precisely
          DriverStation.reportWarning("trying to move right precisely with " + limeX, false);
          Robot.literallyTheDriveTrain.moveMecanumDrive(0.0, 0.0, -RobotMap.VISION_PRECISION_SPEED);
        } else if (limeX < -RobotMap.VISION_PRECISION_RANGE) {
          // start aligning the robot to the left precisely
          DriverStation.reportWarning("trying to move left precisely with " + limeX, false);
          Robot.literallyTheDriveTrain.moveMecanumDrive(0.0, 0.0, RobotMap.VISION_PRECISION_SPEED);
        }
      }
    } else {
      // not doing vision stuff
      SmartDashboard.putBoolean("VisionOn", false);
    }
    lastButtonPress2 = currentPress2;
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
