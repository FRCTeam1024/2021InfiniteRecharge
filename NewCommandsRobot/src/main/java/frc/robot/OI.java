/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
//Do we even really need this anymore?? I think everything is declared in RobotContainer
/**
 * Add your docs here.
 */
public class OI {

    public Joystick leftJoystick;
    public Joystick rightJoystick;
    
    public OI() {
        rightJoystick = new Joystick(2);
        leftJoystick = new Joystick(0);
    }

    public Joystick getLeftJoystick() {
        return leftJoystick;
    }

    public Joystick getRightJoystick() {
        return rightJoystick;
    }
}
