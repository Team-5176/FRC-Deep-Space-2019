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
  boolean lastButtonPress = true;
  boolean state = false;
  int differenceCounter = 0;

  // for pneumatic toggle button 2
  boolean lastButtonPress2 = true;
  boolean state2 = false;
  int differenceCounter2 = 0;


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
      Robot.pneumaticArms.setSolenoid0(false);
      Robot.pneumaticArms.setSolenoid1(true);

      Robot.pneumaticArms.setSolenoid6(false);
      Robot.pneumaticArms.setSolenoid7(true);
    } else {
      Robot.pneumaticArms.setSolenoid0(true);
      Robot.pneumaticArms.setSolenoid1(false);

      Robot.pneumaticArms.setSolenoid6(true);
      Robot.pneumaticArms.setSolenoid7(false);
    }
    lastButtonPress = currentPress;


    boolean currentPress2 = Robot.oi.pilotJoystick.getRawButton(RobotMap.pneumaticToggleButton2);
    boolean isDifferenceBetweenPresses2 = !(currentPress2 == lastButtonPress2);

    if (isDifferenceBetweenPresses2) {
      differenceCounter2++;
      if (differenceCounter2 >= 2) {
        differenceCounter2 = 0;
        state2 = !state2;
      }
    }

    if (state2) {
      Robot.pneumaticArms.setSolenoid2(false);
      Robot.pneumaticArms.setSolenoid3(true);
    } else {
      Robot.pneumaticArms.setSolenoid2(true);
      Robot.pneumaticArms.setSolenoid3(false);
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
