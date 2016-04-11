package org.usfirst.frc.team1014.robot.subsystems;

import org.usfirst.frc.team1014.robot.controls.ControlsManager;

import edu.wpi.first.wpilibj.DigitalOutput;

public class LEDLights
{
	private static LEDLights instance;
	private DigitalOutput bit1, bit2;

	public LEDLights()
	{
		bit1 = new DigitalOutput(ControlsManager.LED_BIT1);
		bit2 = new DigitalOutput(ControlsManager.LED_BIT2);
	}

	/**
	 * returns the current instance of drive train. If none exists, then it creates a new instance.
	 * 
	 * @return instance of the DriveTrain
	 */
	public static LEDLights getInstance()
	{
		if(instance == null)
			instance = new LEDLights();
		return instance;
	}

	public void setLights(LEDState state)
	{
		switch(state)
		{
			case kRED:
				bit1.set(false);
				bit2.set(false);
				break;
			case kBLUE:
				bit1.set(false);
				bit2.set(true);
				break;
			case kSHOOT:
				bit1.set(true);
				bit2.set(false);
				break;
			case kGATHER:
				bit1.set(true);
				bit2.set(true);
				break;
			default:
				bit1.set(false);
				bit2.set(false);
				break;
		}
	}

	public enum LEDState
	{
		kRED, kBLUE, kSHOOT, kGATHER
	}

}
