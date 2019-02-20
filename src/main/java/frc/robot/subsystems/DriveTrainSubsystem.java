/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

// import com.ctre.phoenix.motorcontrol.can.VictorSPX;

// import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;

// import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.misc.MecanumDriveCustom5176;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.DriveTrainMove;

/**
 * Add your docs here.
 */
public class DriveTrainSubsystem extends Subsystem {
  // public VictorSP frontLeftMotor = new VictorSP(RobotMap.frontLeftMotorPort);
  // public VictorSP frontRightMotor = new VictorSP(RobotMap.frontRightMotorPort);
  // public VictorSP backLeftMotor = new VictorSP(RobotMap.backLeftMotorPort);
  // public VictorSP backRightMotor = new VictorSP(RobotMap.backRightMotorPort);

  public WPI_VictorSPX frontLeftMotor = new WPI_VictorSPX(RobotMap.frontLeftCANMotorPort);
  public WPI_VictorSPX frontRightMotor = new WPI_VictorSPX(RobotMap.frontRightCANMotorPort);
  public WPI_VictorSPX backLeftMotor = new WPI_VictorSPX(RobotMap.backLeftCANMotorPort);
  public WPI_VictorSPX backRightMotor = new WPI_VictorSPX(RobotMap.backRightCANMotorPort);

  private MecanumDriveCustom5176 mecanumDrive =
    new MecanumDriveCustom5176(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public void moveMecanumDrive(double joyX, double joyY, double joyZ) { // tradition has been broken :(
    /* also its the function to drive the robot */
    if (joyX < 0.1 && joyX > -0.1) {
      // eliminate small joystick wobble
      joyX = 0;
    }
    if (joyY < 0.1 && joyY > -0.1) {
      // eliminate small joystick wobble
      joyY = 0;
    }

    if (joyX > RobotMap.MAX_DRIVE_SPEED) {
      joyX = RobotMap.MAX_DRIVE_SPEED;
    }

    if (joyX < -RobotMap.MAX_DRIVE_SPEED) {
      joyX = -RobotMap.MAX_DRIVE_SPEED;
    }

    if (joyY > RobotMap.MAX_DRIVE_SPEED) {
      joyY = RobotMap.MAX_DRIVE_SPEED;
    }

    if (joyY < -RobotMap.MAX_DRIVE_SPEED) {
      joyY = -RobotMap.MAX_DRIVE_SPEED;
    }

    mecanumDrive.driveCartesian(joyY, joyX, joyZ);
    // mecanumDrive.driveCartesian(1.0, 1.0, 1.0); // only for the memes :D
    // DEBUG:
    SmartDashboard.putNumber("FL", frontLeftMotor.get());
    SmartDashboard.putNumber("FR", frontRightMotor.get());
    SmartDashboard.putNumber("BL", backLeftMotor.get());
    SmartDashboard.putNumber("BR", backRightMotor.get());
  }

  public void setSpeedFrontMotorsOnly(double speed) {
    
    // if (speed < 0.1 && speed > -0.1) {
    //   speed = 0;
    // }

    // if (speed > RobotMap.maxSpeed) {
    //   speed = RobotMap.maxSpeed;
    // }

    // if (speed < -RobotMap.maxSpeed) {
    //   speed = -RobotMap.maxSpeed;
    // }

    // // drive.arcadeDrive(move, turn);
    // frontLeftMotor.set(speed);
    // frontRightMotor.set(speed);
    // testingMotor.set(1); // only for the memes :D
    // DriverStation.reportWarning("get: " + testingMotor.get() + "", false);

    mecanumDrive.setFrontMotorsOnly(speed);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DriveTrainMove());
  }
}
