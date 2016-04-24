package org.usfirst.frc.team1014.robot.controls;

/**
 * Used to create layouts for different users dependent upon their control preference
 * 
 * Method and variable naming format: (Driver controller function)_(Shooter controller function)
 * 
 * @author Ryan T.
 *
 */
public class ControllerLayout
{
	// the various actions the buttons can map to
	protected int leftDrive_shooter = XboxController.LEFT_STICK_Y,
			rightDrive_articulator = XboxController.RIGHT_STICK_Y;
	protected int adjustBackward_articulatorUp = XboxController.A_BUTTON,
			adjustRight_autoShoot = XboxController.B_BUTTON,
			adjustLeft_servo = XboxController.X_BUTTON,
			adjustForward_articulatorDown = XboxController.Y_BUTTON,
			driveStraight_ = XboxController.LB, _autoGrab = XboxController.RB,
			underVoltClear = XboxController.SELECT,
			_ringLight = XboxController.START,
			left_joy_click = XboxController.LEFT_JOY_CLICK,
			right_joy_click = XboxController.RIGHT_JOY_CLICK;
	protected int layoutchange = XboxController.LEFT_TRIGGER,
			right_trigger = XboxController.RIGHT_TRIGGER;

	public static final int CONTROL_NONE = -1;

	// the actual controller object
	public XboxController controller;

	public ControllerLayout(int controllerPort)
	{
		this.controller = new XboxController(controllerPort, true);
	}

	/**
	 * Gets the value from an analog input based on the current layout and axis to be measured.
	 * 
	 * @param layout
	 *            - the current layout
	 * @param axisID
	 *            - the axis to be measured
	 * @return - the value of the axis to be measured
	 */
	public double getAxisValue(int layout, int axisID)
	{
		if(axisID == ControllerLayout.CONTROL_NONE)
			return 0;

		int sign = axisID / Math.abs(axisID);
		if(controller.onPrimaryLayout && layout == 1)
			return sign * XboxController.deadzone(controller.getRawAxis(Math.abs(axisID)));
		else if(!controller.onPrimaryLayout && layout == 2)
			return sign * XboxController.deadzone(controller.getRawAxis(Math.abs(axisID)));
		else return 0;
	}

	/**
	 * Gets the button values of the controllers based on the layout and button.
	 * 
	 * @param layout
	 *            - the current layout of the controller
	 * @param buttonID
	 *            - the button that needs to be checked
	 * @return - true if the button is pressed, false otherwise
	 */
	public boolean getButtonValue(int layout, int buttonID)
	{
		if(buttonID == ControllerLayout.CONTROL_NONE)
			return false;
		if(controller.onPrimaryLayout && layout == 1)
			return controller.getRawButton(buttonID);
		else if(!controller.onPrimaryLayout && layout == 2)
			return controller.getRawButton(buttonID);
		else return false;
	}

	/*
	 * Getters for the various functions of the robot.
	 */

	public double getLeftDrive_Shooter(int layout)
	{
		return this.getAxisValue(layout, leftDrive_shooter);
	}

	public double getRightDrive_Articulator(int layout)
	{
		return this.getAxisValue(layout, rightDrive_articulator);
	}

	public boolean getAdjustRight_AutoShoot(int layout)
	{
		return this.getButtonValue(layout, adjustRight_autoShoot);
	}

	public boolean getAdjustBackward_ArticulatorUp(int layout)
	{
		return this.getButtonValue(layout, adjustBackward_articulatorUp);
	}

	public boolean getAdjustLeft_Servo(int layout)
	{
		return this.getButtonValue(layout, adjustLeft_servo);
	}

	public boolean getAdjustForward_ArticulatorDown(int layout)
	{
		return this.getButtonValue(layout, adjustForward_articulatorDown);
	}

	public boolean getDriveStraight_(int layout)
	{
		return this.getButtonValue(layout, driveStraight_);
	}

	public boolean get_AutoGrab(int layout)
	{
		return this.getButtonValue(layout, _autoGrab);
	}

	public boolean get_RingLight(int layout)
	{
		return this.getButtonValue(layout, _ringLight);
	}

	public boolean getUnderVoltClear(int layout)
	{
		return this.getButtonValue(layout, this.underVoltClear);
	}

	public boolean getLayoutChange()
	{
		return this.getAxisValue(1, this.layoutchange) > 0.5 || this.getAxisValue(2, this.layoutchange) > 0.5;
	}
}
