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
  public void setSolenoid0(boolean set) {
    solenoid0.set(set);
  }
  public void setSolenoid1(boolean set) {
    solenoid1.set(set);
  }

  public static Solenoid solenoid6 = new Solenoid(6);
  public static Solenoid solenoid7 = new Solenoid(7);
  public void setSolenoid6(boolean set) {
    solenoid6.set(set);
  }
  public void setSolenoid7(boolean set) {
    solenoid7.set(set);
  }

  public static Solenoid solenoid2 = new Solenoid(2);
  public static Solenoid solenoid3 = new Solenoid(3);
  public void setSolenoid2(boolean set) {
    solenoid2.set(set);
  }
  public void setSolenoid3(boolean set) {
    solenoid3.set(set);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new MovePneumaticArms());
  }
}
