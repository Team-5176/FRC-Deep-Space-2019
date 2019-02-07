/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class MovePneumaticArms extends Command {
  public MovePneumaticArms() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.pneumaticArms);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // for pneumatic toggle button
  boolean lastButtonPress = false;
  boolean state = false;
  int differenceCounter = 0;

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    boolean currentPress = Robot.oi.pilotJoystick.getRawButton(RobotMap.pneumaticToggleButton);
    boolean isDifferenceBetweenPresses = !(currentPress == lastButtonPress);

    if (isDifferenceBetweenPresses) {
      differenceCounter++;
      if (differenceCounter >= 2) {
        differenceCounter = 0;
        state = !state;
      }
    }

    if (state) {
      Robot.pneumaticArms.solenoid0.set(true);
      Robot.pneumaticArms.solenoid1.set(false);
    } else {
      Robot.pneumaticArms.solenoid0.set(false);
      Robot.pneumaticArms.solenoid1.set(true);
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
  }
}
