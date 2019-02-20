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
    panicModeFront = false;
    panicModeRear = false;
  
    // TODO THE FOLLOWING TESTING CODE WILL NEED TO BE REMOVED LATER
    testMoveFrontAt40 = false;
    testMoveFrontAt5 = false;
    testMoveRearAt40 = false;
    testMoveRearAt5 = false;
  
    stillMoveAutoFront = true;
    manualMoveFront = 0.0;
    stillMoveAutoRear = true;
    manualMoveRear = 0.0;

    amIDoingTheClimb = false;
    // END TESTING CODE
  }

  public boolean panicModeFront = false;
  public boolean panicModeRear = false;

  // TODO THE FOLLOWING TESTING CODE WILL NEED TO BE REMOVED LATER
  boolean testMoveFrontAt40 = false;
  boolean testMoveFrontAt5 = false;
  boolean testMoveRearAt40 = false;
  boolean testMoveRearAt5 = false;

  boolean stillMoveAutoFront = true;
  double manualMoveFront = 0.0;
  boolean stillMoveAutoRear = true;
  double manualMoveRear = 0.0;

  boolean amIDoingTheClimb = false;
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
    if (Robot.oi.coPilotJoystick.getRawButton(RobotMap.LOGITECH_A)) { // A
      testMoveFrontAt40 = true;
      testMoveFrontAt5 = false;
      testMoveRearAt40 = true;
      testMoveRearAt5 = false;
    } else if (Robot.oi.coPilotJoystick.getRawButton(RobotMap.LOGITECH_Y)) { // Y
      testMoveFrontAt40 = false;
      testMoveFrontAt5 = true;
      testMoveRearAt40 = false;
      testMoveRearAt5 = true;
    } else if (Robot.oi.coPilotJoystick.getRawButton(RobotMap.LOGITECH_B)) { // B
      testMoveFrontAt40 = false;
      testMoveFrontAt5 = false;
      testMoveRearAt40 = false;
      testMoveRearAt5 = false;
    }

    if (Robot.oi.pilotJoystick.getRawButton(5)) {
      
    }

    // Code for the front boi
    if (testMoveFrontAt40) {
      if (!Robot.climberSystem.limitSwitchFront.get()) {
      // if (false) {
        // the limit switch is hit
        testMoveFrontAt40 = false;
        testMoveFrontAt5 = true;
      } else {
        Robot.climberSystem.climberFrontMotor.set(-RobotMap.GO_CLIMB_SPEED_FRONT); 
      }
      // Robot.climberSystem.climberRearMotor.set(-RobotMap.GO_CLIMB_SPEED_REAR);
    }
    if (testMoveFrontAt5) {
      // first check for the joystick. if it is nothing, do the panic speed
      double currentCoPilotFrontRawAxis = Robot.oi.coPilotJoystick.getRawAxis(2); // left trigger
      if (currentCoPilotFrontRawAxis < 0.1) {
        manualMoveFront = 0.0;
      } else {
        manualMoveFront = currentCoPilotFrontRawAxis;
        stillMoveAutoFront = false;
      }
      if (stillMoveAutoFront) {
        Robot.climberSystem.climberFrontMotor.set(-RobotMap.PANIC_CLIMB_SPEED_FRONT);
      } else {
        Robot.climberSystem.climberFrontMotor.set(currentCoPilotFrontRawAxis);
      }
      // Robot.climberSystem.climberRearMotor.set(-RobotMap.PANIC_CLIMB_SPEED_REAR);
    }
    if (!testMoveFrontAt40 && !testMoveFrontAt5) {
      Robot.climberSystem.climberFrontMotor.set(0.0);
      // Robot.climberSystem.climberRearMotor.set(0.0);
    }

    // Code for the rear boi
    if (testMoveRearAt40) {
      // Robot.climberSystem.climberFrontMotor.set(-RobotMap.GO_CLIMB_SPEED_FRONT); 
      if (!Robot.climberSystem.limitSwitchRear.get()) {
      // if (false) {
        // the limit switch is hit
        testMoveRearAt40 = false;
        testMoveRearAt5 = true;
      } else {
        Robot.climberSystem.climberRearMotor.set(-RobotMap.GO_CLIMB_SPEED_REAR);
      }
    }
    if (testMoveRearAt5) {
      // first check for the joystick. if it is nothing, do the panic speed
      double currentCoPilotRearRawAxis = Robot.oi.coPilotJoystick.getRawAxis(3); // right trigger
      if (currentCoPilotRearRawAxis < 0.1) {
        manualMoveRear = 0.0;
      } else {
        manualMoveRear = currentCoPilotRearRawAxis;
        stillMoveAutoRear = false;
      }
      if (stillMoveAutoRear) {
        Robot.climberSystem.climberRearMotor.set(-RobotMap.PANIC_CLIMB_SPEED_REAR);
      } else {
        Robot.climberSystem.climberRearMotor.set(currentCoPilotRearRawAxis);
      }
      // Robot.climberSystem.climberFrontMotor.set(-RobotMap.PANIC_CLIMB_SPEED_FRONT);
      // Robot.climberSystem.climberRearMotor.set(-RobotMap.PANIC_CLIMB_SPEED_REAR);
    }
    if (!testMoveRearAt40 && !testMoveRearAt5) {
      // Robot.climberSystem.climberFrontMotor.set(0.0);
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
