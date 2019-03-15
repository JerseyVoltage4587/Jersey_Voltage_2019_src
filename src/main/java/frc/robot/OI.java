/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.util.JoyButton;
import frc.robot.commands.StartVisionDrive;
import frc.robot.subsystems.Intake.IntakeControlState;
import frc.robot.commands.ClimbHalfOn;
import frc.robot.commands.ClimbRest;
import frc.robot.commands.ClimbUp;
import frc.robot.commands.SetArmSetpoint;
import frc.robot.commands.SetLiftSetpoint;
import frc.robot.commands.SetCameraMode;
import frc.robot.commands.StartOpenLoop;
import frc.robot.commands.StartSimpleVision;
import frc.robot.commands.SetIntakeState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
//import utility.JoyButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	private static OI mInstance = null;
	private Joystick stick1;
	Button	  buttonA1, buttonB1, buttonX1, buttonY1, leftBumper1, rightBumper1,startButton1;
	JoyButton leftTrigger1, rightTrigger1;
	Joystick  stick2;
	Button	  buttonA2, buttonB2, buttonX2, buttonY2, leftBumper2, rightBumper2;
	JoyButton leftTrigger2, rightTrigger2;
	Joystick  driverStation;
	Button    toggleSwitch0, toggleSwitch1, toggleSwitch2, toggleSwitch3, toggleSwitch4, tinesSwitch, debugSwitch;
	Button	  count0Button1, count0Button2, count1Button1, count1Button2, count2Button1, count2Button2, count3Button1, count3Button2;

	// Return the singleton OI object, creating it if necessary.
	// Creating the object is synchronized, just in case two threads end up calling simultaneously.
	public static OI getInstance()
	{
		if(mInstance == null) {
			synchronized ( OI.class ) {
				mInstance = new OI();
			}
		}
		return mInstance;
	}
	
	public OI()
	{
		//stick1 = new Joystick(1);
		stick1			= new Joystick(1);
    	buttonA1		= new JoystickButton(stick1, 1);
    	buttonB1		= new JoystickButton(stick1, 2);
    	buttonX1		= new JoystickButton(stick1, 3);
    	buttonY1		= new JoystickButton(stick1, 4);
    	leftBumper1 	= new JoystickButton(stick1, 5);
    	leftTrigger1	= new JoyButton(stick1, JoyButton.JoyDir.DOWN, 2);
    	rightBumper1	= new JoystickButton(stick1, 6);
    	rightTrigger1	= new JoyButton(stick1, JoyButton.JoyDir.DOWN, 3);
    	startButton1	= new JoystickButton(stick1, 8);
    	
    	stick2			= new Joystick(2);
    	buttonA2		= new JoystickButton(stick2, 1);
    	buttonB2		= new JoystickButton(stick2, 2);
    	buttonX2		= new JoystickButton(stick2, 3);
    	buttonY2		= new JoystickButton(stick2, 4);
    	leftBumper2 	= new JoystickButton(stick2, 5);
    	leftTrigger2	= new JoyButton(stick2, JoyButton.JoyDir.DOWN, 2);
    	rightBumper2	= new JoystickButton(stick2, 6);
    	rightTrigger2	= new JoyButton(stick2, JoyButton.JoyDir.DOWN, 3);
    	
    	driverStation   = new Joystick(0);
    	toggleSwitch0   = new JoystickButton(driverStation, 1);
    	toggleSwitch1   = new JoystickButton(driverStation, 2);
    	tinesSwitch   	= new JoystickButton(driverStation, 3);
    	debugSwitch 	= new JoystickButton(driverStation, 7);
    	// turn auto off button 14
    	count0Button1 	= new JoystickButton(driverStation, 11);
    	count0Button2 	= new JoystickButton(driverStation, 10);
    	count1Button1 	= new JoystickButton(driverStation, 13);
    	count1Button2 	= new JoystickButton(driverStation, 12);
    	count2Button1 	= new JoystickButton(driverStation, 15);
    	count2Button2 	= new JoystickButton(driverStation, 16);
    	count3Button1 	= new JoystickButton(driverStation, 9);
    	count3Button2 	= new JoystickButton(driverStation, 8);
    	
    	//System.out.println("OI start");  println is evil
    	/*buttonA1.whenPressed(new SetArmSetpoint(-90));
		buttonB1.whenPressed(new SetArmSetpoint(0));
		buttonY1.whenPressed(new SetArmSetpoint(90));
		*///buttonX1.whenPressed(new StartSimpleVision());
		
		/*buttonA1.whenPressed(new SetLiftSetpoint(0.0));
		buttonB1.whenPressed(new SetLiftSetpoint(1.0));
		buttonY1.whenPressed(new SetLiftSetpoint(4.5));
		*/
		/*buttonA1.whenPressed(new ClimbUp());
		buttonB1.whenPressed(new ClimbHalfOn());
		buttonY1.whenPressed(new ClimbRest());
		*/
		/*buttonA1.whenPressed(new SetIntakeState(IntakeControlState.INTAKE_HATCH));
		buttonB1.whenPressed(new SetIntakeState(IntakeControlState.OFF));
		buttonX1.whenPressed(new SetIntakeState(IntakeControlState.HOLD_HATCH));
		*/
		buttonA1.whenPressed(new SetIntakeState(IntakeControlState.INTAKE_BALL));
		buttonB1.whenPressed(new SetIntakeState(IntakeControlState.OFF));
	}

	// Get the value of the "drive" stick.
	public double getDrive()
	{
		return -1 * stick1.getRawAxis(1);
	}

	// Get the value of the "turn" stick.
	public double getTurn()
	{
		return stick1.getRawAxis(4);
	}
	
	public int getPOV() {
		return stick1.getPOV();
	}
}
