/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.ClimberCommand;

/**
 * Add your docs here.
 */
public class Climber extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public DigitalInput limitSwitchFront = new DigitalInput(RobotMap.climberFrontLimitSwitchPort);
  public DigitalInput limitSwitchRear = new DigitalInput(RobotMap.climberRearLimitSwitchPort);
  public VictorSP climberFrontMotor = new VictorSP(RobotMap.frontClimberMotorPort);
  public VictorSP climberRearMotor = new VictorSP(RobotMap.rearClimberMotorPort);

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new ClimberCommand());
  }
}
