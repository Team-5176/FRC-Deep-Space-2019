/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class ClimberSequence extends Command {

  private static final int SECOND_TO_MICRO_MULTIPLIER = 1000000;

  public ClimberSequence() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.climberSystem);
  }

  long timeAtInit;
  boolean hasRun;
  double frontLiftGoSpeed;

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // timeAtInit = RobotController.getFPGATime();
    timeAtInit = -1;
    hasRun = false;
    frontLiftGoSpeed = RobotMap.GO_CLIMB_SPEED_FRONT;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (!hasRun) {
      hasRun = true;
      frontLiftGoSpeed = RobotMap.GO_CLIMB_SPEED_FRONT;
      timeAtInit = RobotController.getFPGATime();
      // while (timeAtInit + (2 * SECOND_TO_MICRO_MULTIPLIER) >= RobotController.getFPGATime()) {
      //   Robot.testSubsystem.testingMotor2.set(0.5);
      // }
      // Robot.testSubsystem.testingMotor2.set(0.0);

      // we are in
      // put both climb motors on
      while (Robot.climberSystem.limitSwitchFront.get() ||
             Robot.climberSystem.limitSwitchRear.get()) {
        // if we are in this loop, then one or both of the
        // limit switches are NOT activated.
        if (!Robot.climberSystem.limitSwitchFront.get()) {
          // the front limit switch is activated
          Robot.climberSystem.climberFrontMotor.set(-RobotMap.PANIC_CLIMB_SPEED_FRONT);
        } else {
          // the front limit switch is NOT activated
          Robot.climberSystem.climberFrontMotor.set(-frontLiftGoSpeed);
        }
        if (!Robot.climberSystem.limitSwitchRear.get()) {
          // the rear limit switch is activated
          Robot.climberSystem.climberRearMotor.set(-RobotMap.PANIC_CLIMB_SPEED_REAR);
        } else {
          // the rear limit switch is NOT activated
          Robot.climberSystem.climberRearMotor.set(-RobotMap.GO_CLIMB_SPEED_REAR);
        }
        if (RobotController.getFPGATime() % 100000 == 0) {
          // this code should^TM run every 1/10 of a second
          if (frontLiftGoSpeed < 0.26) {
            frontLiftGoSpeed += 0.025;
          }
          SmartDashboard.putNumber("frontLift", frontLiftGoSpeed);
        }
      }
      // now both of the arms are fully extended and
      // both limit switches are activated

      Robot.climberSystem.climberRearMotor.set(-RobotMap.PANIC_CLIMB_SPEED_REAR);
      Robot.climberSystem.climberFrontMotor.set(-RobotMap.PANIC_CLIMB_SPEED_FRONT);

      // we now need to drive forward using the
      // motor/wheel on the bottom of the climber

      // get current time (in microseconds) and
      // run the motor at x% (25) power for x (2) seconds
      long timeAtStartToDriveForward = RobotController.getFPGATime();
      while (timeAtStartToDriveForward + 4000000
             >= RobotController.getFPGATime()) {
        Robot.testSubsystem.setSpeed2(0.25);
      }
      // then set the bottom wheel to 0% power
      Robot.testSubsystem.setSpeed2(0.0);

      // now the front wheels are over the box
      // we will run the front sensors at 30% 
      // for 3 seconds

      long timeAtStartToMoveFrontThingUp = RobotController.getFPGATime();
      while (timeAtStartToMoveFrontThingUp + 5300000 >= RobotController.getFPGATime()) {
        Robot.climberSystem.climberFrontMotor.set(0.30);
      }
      Robot.climberSystem.climberFrontMotor.set(0.0);

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

      // now we pull up the rear climber leg for 2 seconds at 30%
      long timeAtPullUpRearClimberLeg = RobotController.getFPGATime();
      while (timeAtPullUpRearClimberLeg + 4500000 >= RobotController.getFPGATime()) {
        Robot.climberSystem.climberRearMotor.set(0.30);
      }
      Robot.climberSystem.climberRearMotor.set(0.0);

      // at this point the climb should be done
    }
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
    DriverStation.reportError("help", false);
  }
}
