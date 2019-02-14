/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;

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

  // for vacuum toggle button
  boolean lastButtonPress = true;
  boolean motorState = false;
  int differenceCounter = 0;

  
  
  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // begin limelight testing code
    double limeX = RobotMap.limelightTx.getDouble(0.0);
    double limeY = RobotMap.limelightTy.getDouble(0.0);
    double limeA = RobotMap.limelightTa.getDouble(0.0);
    boolean limeHasTarget = RobotMap.limelightTv.getDouble(0.0) == 1;

    SmartDashboard.putNumber("LimelightX", limeX);
    SmartDashboard.putNumber("LimelightY", limeY);
    SmartDashboard.putNumber("LimelightA", limeA);
    SmartDashboard.putBoolean("LimelightHas", limeHasTarget);
    // end limelight testing code
    
    double move = -Robot.oi.coPilotJoystick.getY();

    // 3 is the Z rotate axis
    // double move1 = -Robot.oi.coPilotJoystick.getRawAxis(3);
    // DriverStation.reportWarning(Robot.oi.pilotJoystick.getY() + "", false);
    Robot.testSubsystem.setSpeed2(move);
    // Robot.testSubsystem.setSpeed(move);
    // Robot.testSubsystem.setSpeed(move1);
    // Robot.testSubsystem.setSpeed(0.11); // port 0
    // Robot.testSubsystem.testingMotor.set(0.3); // manual, port 0
    // Robot.testSubsystem.setSpeed2(1); // port 5
    
    boolean currentPress = Robot.oi.pilotJoystick.getRawButton(RobotMap.motorToggleButton);
    boolean isDifferenceBetweenPresses = !(currentPress == lastButtonPress);


    // DriverStation.reportWarning(Robot.oi.pilotJoystick.getRawButton(2) + "", false);
    if (isDifferenceBetweenPresses) {
      differenceCounter++;
      if (differenceCounter >= 2) {
        differenceCounter = 0;
        motorState = !motorState;
      }
    }
    if (motorState) {
      // DriverStation.reportWarning("it is on", false);
      Robot.testSubsystem.testingMotor.set(RobotMap.vacuumSpeed); // port 0; manual control
      Robot.testSubsystem.secondVacuum.set(RobotMap.vacuumSpeed); // port 8; manual control
      Robot.testSubsystem.thirdVacuum.set(RobotMap.vacuumSpeed); // port 9; manual control
    } else {
      // DriverStation.reportWarning("no go", false);
      Robot.testSubsystem.testingMotor.set(0.0); // port 0; manual control
      Robot.testSubsystem.secondVacuum.set(0.0); // port 8; manual control
      Robot.testSubsystem.thirdVacuum.set(0.0); // port 9; manual control
    }
    lastButtonPress = currentPress;

    

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
