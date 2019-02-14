/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class ClimberCommand extends Command {
  public ClimberCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.climberSystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  public boolean panicModeFront = false;
  public boolean panicModeRear = false;

  // TODO THE FOLLOWING TESTING CODE WILL NEED TO BE REMOVED LATER
  boolean testMoveAt40 = false;
  boolean testMoveAt5 = false;
  // END TESTING CODE

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    boolean limitSwitchFrontStatus = Robot.climberSystem.limitSwitchFront.get();
    boolean limitSwitchRearStatus = Robot.climberSystem.limitSwitchRear.get();
    // I believe that the limitSwitchStatus is set to true when not depressed,
    // but false when it is depressed. This is only because of the way the switch
    // is wired.
    SmartDashboard.putBoolean("limitSwitchFront", limitSwitchFrontStatus);
    SmartDashboard.putBoolean("limitSwitchRear", limitSwitchRearStatus);
    if (!limitSwitchFrontStatus) {
      panicModeFront = true;
    }
    if (!limitSwitchRearStatus) {
      panicModeRear = true;
    }
    if (panicModeFront) {
      // once we have activated panic mode we want to check for the manual override button
      if (Robot.oi.coPilotJoystick.getRawButton(RobotMap.manualFrontClimbOverride)) {
        panicModeFront = false;
        
      } else {
        // panic mode == true; manual override == false.
        // TODO: Robot.climberSystem.climberFrontMotor.set(RobotMap.PANIC_CLIMB_SPEED);
      }
    } else {
      // panic mode == false.

    }
    // no matter what, if we have the manual override button pressed
    // we want the climber to start moving UP (so that side of the robot would go down)
    if (Robot.oi.coPilotJoystick.getRawButton(RobotMap.manualFrontClimbOverride)) {
      Robot.climberSystem.climberFrontMotor.set(-0.2); // TODO: value subject to change
    }
    if (panicModeRear) {

    } else {

    }

    // TODO THE FOLLOWING TESTING CODE WILL NEED TO BE REMOVED LATER
    if (Robot.oi.coPilotJoystick.getRawButton(2)) { // A
      testMoveAt40 = true;
      testMoveAt5 = false;
    } else if (Robot.oi.coPilotJoystick.getRawButton(4)) { // Y
      testMoveAt40 = false;
      testMoveAt5 = true;
    } else if (Robot.oi.coPilotJoystick.getRawButton(3)) { // B
      testMoveAt40 = false;
      testMoveAt5 = false;
    }
    if (testMoveAt40) {
      Robot.climberSystem.climberFrontMotor.set(-0.43);
      Robot.climberSystem.climberRearMotor.set(-0.4);
    } else if (testMoveAt5) {
      Robot.climberSystem.climberFrontMotor.set(-RobotMap.PANIC_CLIMB_SPEED_FRONT);
      Robot.climberSystem.climberRearMotor.set(-RobotMap.PANIC_CLIMB_SPEED_REAR);
    } else {
      Robot.climberSystem.climberFrontMotor.set(0.0);
      Robot.climberSystem.climberRearMotor.set(0.0);
    }
    // END TESTING CODE
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
