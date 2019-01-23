/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.DriverStation;

// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
import frc.robot.commands.TestMoveMotor;

/**
 * Add your docs here.
 */
public class TestingSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // public WPI_TalonSRX leftMaster = new WPI_TalonSRX(RobotMap.leftMasterPort);
  // public WPI_TalonSRX leftSlave = new WPI_TalonSRX(RobotMap.leftSlavePort);
  // public WPI_TalonSRX rightMaster = new WPI_TalonSRX(RobotMap.rightMasterPort);
  // public WPI_TalonSRX rightSlave = new WPI_TalonSRX(RobotMap.rightSlavePort);
  public VictorSP testingMotor = new VictorSP(RobotMap.testMotorPort);

  // public DifferentialDrive drive = new DifferentialDrive(leftMaster, rightMaster);

  public TestingSubsystem() {
    // leftSlave.follow(leftMaster);
    // rightSlave.follow(rightMaster);
  }

  public void setSpeed(double speed) {
    
    if (speed < 0.1 && speed > -0.1) {
      speed = 0;
    }

    if (speed > RobotMap.maxSpeed) {
      speed = RobotMap.maxSpeed;
    }

    if (speed < -RobotMap.maxSpeed) {
      speed = -RobotMap.maxSpeed;
    }

    // drive.arcadeDrive(move, turn);
    testingMotor.set(speed);
    // testingMotor.set(1); // only for the memes :D
    // DriverStation.reportWarning("get: " + testingMotor.get() + "", false);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new TestMoveMotor());
  }
}
