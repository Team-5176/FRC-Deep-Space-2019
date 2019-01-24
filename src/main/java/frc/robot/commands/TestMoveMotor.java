/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class TestMoveMotor extends Command {
  public TestMoveMotor() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.testSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }
  
  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
    double move = -Robot.oi.pilotJoystick.getY();

    // 3 is the Z rotate axis
    double move1 = -Robot.oi.coPilotJoystick.getRawAxis(3);
    // DriverStation.reportWarning(Robot.oi.pilotJoystick.getY() + "", false);
    // Robot.testSubsystem.setSpeed2(move);
    // Robot.testSubsystem.setSpeed(move1);
    // Robot.testSubsystem.setSpeed(0.11); // port 0
    Robot.testSubsystem.testingMotor.set(0.4); // manual, port 0
    Robot.testSubsystem.setSpeed2(1); // port 5

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
