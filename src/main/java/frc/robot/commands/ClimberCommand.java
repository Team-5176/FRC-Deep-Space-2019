/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
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

    amIDoingTheClimbFront = false;
    amIDoingTheClimbRear = false;

    haveIRetractedTheFrontYet = false;
    haveIRetractedTheRearYet = false;
    haveIDrivenFirst = false;
    haveIDrivenSecond = false;
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

  boolean amIDoingTheClimbFront = false;
  boolean amIDoingTheClimbRear = false;

  boolean haveIRetractedTheFrontYet = false;
  boolean haveIRetractedTheRearYet = false;
  boolean haveIDrivenFirst = false;
  boolean haveIDrivenSecond = false;
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
    // if (!limitSwitchFrontStatus) {
    //   panicModeFront = true;
    // }
    // if (!limitSwitchRearStatus) {
    //   panicModeRear = true;
    // }
    // if (panicModeFront) {
    //   // once we have activated panic mode we want to check for the manual override button
    //   if (Robot.oi.coPilotJoystick.getRawButton(RobotMap.manualFrontClimbOverride)) {
    //     panicModeFront = false;
        
    //   } else {
    //     // panic mode == true; manual override == false.
    //     // TODO: Robot.climberSystem.climberFrontMotor.set(RobotMap.PANIC_CLIMB_SPEED);
    //   }
    // } else {
    //   // panic mode == false.

    // }
    // // no matter what, if we have the manual override button pressed
    // // we want the climber to start moving UP (so that side of the robot would go down)
    // if (Robot.oi.coPilotJoystick.getRawButton(RobotMap.manualFrontClimbOverride)) {
    //   Robot.climberSystem.climberFrontMotor.set(-0.2); // TODO: value subject to change
    // }
    // if (panicModeRear) {

    // } else {

    // }

    // TODO THE FOLLOWING TESTING CODE WILL NEED TO BE REMOVED LATER
    // if (Robot.oi.coPilotJoystick.getRawButton(RobotMap.LOGITECH_A)) { // A
    //   testMoveFrontAt40 = true;
    //   testMoveFrontAt5 = false;
    //   testMoveRearAt40 = true;
    //   testMoveRearAt5 = false;
    // } else if (Robot.oi.coPilotJoystick.getRawButton(RobotMap.LOGITECH_Y)) { // Y
    //   testMoveFrontAt40 = false;
    //   testMoveFrontAt5 = true;
    //   testMoveRearAt40 = false;
    //   testMoveRearAt5 = true;
    // } else if (Robot.oi.coPilotJoystick.getRawButton(RobotMap.LOGITECH_B)) { // B
    //   testMoveFrontAt40 = false;
    //   testMoveFrontAt5 = false;
    //   testMoveRearAt40 = false;
    //   testMoveRearAt5 = false;
    // }

    // if (Robot.oi.pilotJoystick.getRawButton(5)) {
      
    // }

    if (Robot.oi.pilotJoystick.getRawButton(RobotMap.LOGITECH_LB)) {
      // once you push either LB or RB, the climb cant be reversed
      amIDoingTheClimbFront = true;

      testMoveFrontAt40 = true;
    } else {
      testMoveFrontAt40 = false;
    }

    if (Robot.oi.pilotJoystick.getRawButton(RobotMap.LOGITECH_RB)) {
      // once you push either LB or RB, the climb cant be reversed
      amIDoingTheClimbRear = true;

      testMoveRearAt40 = true;
    } else {
      testMoveRearAt40 = false;
    }

    if (amIDoingTheClimbFront && !testMoveFrontAt40) {
      // if we are doing the climb and not actively moving up,
      // set the thing to freeze at 5
      testMoveFrontAt5 = true;
    } else {
      testMoveFrontAt5 = false;
    }

    if (amIDoingTheClimbRear && !testMoveRearAt40) {
      // if we are doing the climb and not actively moving up,
      // set the thing to freeze at 5
      testMoveRearAt5 = true;
    } else {
      testMoveRearAt5 = false;
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
      // double currentCoPilotFrontRawAxis = Robot.oi.coPilotJoystick.getRawAxis(2); // left trigger
      // if (currentCoPilotFrontRawAxis < 0.1) {
        // manualMoveFront = 0.0;
      // } else {
      //   manualMoveFront = currentCoPilotFrontRawAxis;
      //   stillMoveAutoFront = false;
      // }
      // if (stillMoveAutoFront) {
        Robot.climberSystem.climberFrontMotor.set(-RobotMap.PANIC_CLIMB_SPEED_FRONT);
      // } else {
      //   Robot.climberSystem.climberFrontMotor.set(currentCoPilotFrontRawAxis);
      // }
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
      // double currentCoPilotRearRawAxis = Robot.oi.coPilotJoystick.getRawAxis(3); // right trigger
      // if (currentCoPilotRearRawAxis < 0.1) {
      //   manualMoveRear = 0.0;
      // } else {
      //   manualMoveRear = currentCoPilotRearRawAxis;
      //   stillMoveAutoRear = false;
      // }
      // if (stillMoveAutoRear) {
        Robot.climberSystem.climberRearMotor.set(-RobotMap.PANIC_CLIMB_SPEED_REAR);
      // } else {
      //   Robot.climberSystem.climberRearMotor.set(currentCoPilotRearRawAxis);
      // }
      // Robot.climberSystem.climberFrontMotor.set(-RobotMap.PANIC_CLIMB_SPEED_FRONT);
      // Robot.climberSystem.climberRearMotor.set(-RobotMap.PANIC_CLIMB_SPEED_REAR);
    }
    if (!testMoveRearAt40 && !testMoveRearAt5) {
      // Robot.climberSystem.climberFrontMotor.set(0.0);
      Robot.climberSystem.climberRearMotor.set(0.0);
    }

    if (Robot.oi.pilotJoystick.getRawButton(RobotMap.LOGITECH_B)) {
      // this is the code where we retract the front climb arm
      amIDoingTheClimbFront = false; // we no longer want to hold the front arm at 5
      if (!haveIRetractedTheFrontYet) {
        // only run once if the front has not been retracted yet
        haveIRetractedTheFrontYet = true;
        long timeAtStartOfFrontRetract = RobotController.getFPGATime();
        while (timeAtStartOfFrontRetract + 5300000 >= RobotController.getFPGATime()) {
          Robot.climberSystem.climberFrontMotor.set(0.30);
        }
        Robot.climberSystem.climberFrontMotor.set(0.0);
      }
    }

    if (Robot.oi.pilotJoystick.getRawButton(RobotMap.LOGITECH_X)) {
      // this is the code where we retract the rear climb arm
      amIDoingTheClimbRear = false; // we no longer want to hold the front arm at 5
      if (!haveIRetractedTheRearYet) {
        // only run once if the front has not been retracted yet
        haveIRetractedTheRearYet = true;
        long timeAtStartOfRearRetract = RobotController.getFPGATime();
        while (timeAtStartOfRearRetract + 4500000 >= RobotController.getFPGATime()) {
          Robot.climberSystem.climberRearMotor.set(0.30);
        }
        Robot.climberSystem.climberRearMotor.set(0.0);
      }
    }

    if (Robot.oi.pilotJoystick.getRawButton(RobotMap.LOGITECH_A)) {
      // this is the code where we drive the back for the first time
      if (!haveIDrivenFirst) {
        haveIDrivenFirst = true;
        long timeAtStartToDriveForward = RobotController.getFPGATime();
        while (timeAtStartToDriveForward + 4000000
             >= RobotController.getFPGATime()) {
        Robot.testSubsystem.setSpeed2(0.25);
      }
      // then set the bottom wheel to 0% power
      Robot.testSubsystem.setSpeed2(0.0);
      }

    }

    if (Robot.oi.pilotJoystick.getRawButton(RobotMap.LOGITECH_Y)) {
      // this is the code where we drive the back for the first time
      if (!haveIDrivenSecond) {
        haveIDrivenSecond = true;
        
        // now we tell the drive train to shut the hell up
        DriveTrainMove.shouldIDrive = false;

        // now we move the bottom wheels at 50% and the front
        // wheels on 10% for 2 seconds
        long timeAtMoveOnBoxAgain = RobotController.getFPGATime();
        while (timeAtMoveOnBoxAgain + 7000000 >= RobotController.getFPGATime()) {
          Robot.testSubsystem.setSpeed2(0.25);
          Robot.literallyTheDriveTrain.setSpeedFrontMotorsOnly(0.1);
        }
        Robot.testSubsystem.setSpeed2(0.0);
        Robot.literallyTheDriveTrain.setSpeedFrontMotorsOnly(0.0);
      }

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
