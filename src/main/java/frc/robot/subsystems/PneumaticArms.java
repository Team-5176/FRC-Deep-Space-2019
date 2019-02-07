/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.MovePneumaticArms;

/**
 * Add your docs here.
 */
public class PneumaticArms extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public static Solenoid solenoid0 = new Solenoid(0);
  public static Solenoid solenoid1 = new Solenoid(1);

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new MovePneumaticArms());
  }
}
