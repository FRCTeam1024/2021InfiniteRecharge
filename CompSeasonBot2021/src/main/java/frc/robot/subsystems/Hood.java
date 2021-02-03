/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {

  private final Solenoid hoodSolenoid = new Solenoid(4); //Lets move all device IDs to Constants.java eventually


  /**
   * Creates a new Hood
   */
  public Hood() {
    

  }

  public void extendHood(){  //These may need their own subsystem to actuate while spinning the shooter
    hoodSolenoid.set(true);
  }
  public void retractHood(){
    hoodSolenoid.set(false);
  }

  @Override
  public void periodic() {
    
  }
}
