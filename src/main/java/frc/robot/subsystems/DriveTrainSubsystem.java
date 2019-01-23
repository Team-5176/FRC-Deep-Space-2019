/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class DriveTrainSubsystem extends Subsystem {
  public static VictorSP frontLeftMotor = new VictorSP(RobotMap.frontLeftMotorPort);
  public static VictorSP frontRightMotor = new VictorSP(RobotMap.frontRightMotorPort);
  public static VictorSP backLeftMotor = new VictorSP(RobotMap.backLeftMotorPort);
  public static VictorSP backRightMotor = new VictorSP(RobotMap.backRightMotorPort);

  public static MecanumDrive mecanumDrive =
    new MecanumDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
