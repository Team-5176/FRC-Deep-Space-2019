/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.MovePneumaticArms;

/**
 * Add your docs here.
 */
public class PneumaticArms extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  // public static Solenoid solenoid0 = new Solenoid(0);
  // public static Solenoid solenoid1 = new Solenoid(1);
  // public void setSolenoid0(boolean set) {
  //   solenoid0.set(set);
  // }
  // public void setSolenoid1(boolean set) {
  //   solenoid1.set(set);
  // }

  // public static Solenoid solenoid6 = new Solenoid(6);
  // public static Solenoid solenoid7 = new Solenoid(7);
  // public void setSolenoid6(boolean set) {
  //   solenoid6.set(set);
  // }
  // public void setSolenoid7(boolean set) {
  //   solenoid7.set(set);
  // }

  // public static Solenoid solenoid2 = new Solenoid(2);
  // public static Solenoid solenoid3 = new Solenoid(3);
  // public void setSolenoid2(boolean set) {
  //   solenoid2.set(set);
  // }
  // public void setSolenoid3(boolean set) {
  //   solenoid3.set(set);
  // }

  public Solenoid vacuumSolenoid0A = new Solenoid(RobotMap.VACUUM_SOLENOID_0_A);
  public Solenoid vacuumSolenoid0B = new Solenoid(RobotMap.VACUUM_SOLENOID_0_B);
  public Solenoid vacuumSolenoid1A = new Solenoid(RobotMap.VACUUM_SOLENOID_1_A);
  public Solenoid vacuumSolenoid1B = new Solenoid(RobotMap.VACUUM_SOLENOID_1_B);
  public Solenoid vacuumSolenoid2A = new Solenoid(RobotMap.VACUUM_SOLENOID_2_A);
  public Solenoid vacuumSolenoid2B = new Solenoid(RobotMap.VACUUM_SOLENOID_2_B);
  public void setVacuumSolenoid0A(boolean set) {
    vacuumSolenoid0A.set(set);
  }
  public void setVacuumSolenoid0B(boolean set) {
    vacuumSolenoid0B.set(set);
  }
  public void setVacuumSolenoid1A(boolean set) {
    vacuumSolenoid1A.set(set);
  }
  public void setVacuumSolenoid1B(boolean set) {
    vacuumSolenoid1B.set(set);
  }
  public void setVacuumSolenoid2A(boolean set) {
    vacuumSolenoid2A.set(set);
  }
  public void setVacuumSolenoid2B(boolean set) {
    vacuumSolenoid2B.set(set);
  }
  public Solenoid armSolenoid0A = new Solenoid(RobotMap.ARM_SOLENOID_0_A);
  public Solenoid armSolenoid0B = new Solenoid(RobotMap.ARM_SOLENOID_0_B);
  public void setArmSolenoid0A(boolean set) {
    armSolenoid0A.set(set);
  }
  public void setArmSolenoid0B(boolean set) {
    armSolenoid0B.set(set);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new MovePneumaticArms());
  }
}
