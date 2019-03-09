package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GamepadSwerve extends SwerveDrive {

	private XboxController driveController;
	private JoystickButton driverGamepadGyroResetButton;
	private JoystickButton driverGamepadTuningActivation;
	private Joystick leftDriveStick;
	private Joystick rightDriveStick;
	private JoystickButton joystickRobotFrontCargoButton;
	private JoystickButton joystickRobotFrontHatchButton;
	private JoystickButton joystickGyroResetButton;

    public GamepadSwerve(AHRS ahrs, XboxController driveController, Joystick leftDriveStick, Joystick rightDriveStick) {
		super(ahrs);
		this.driveController = driveController;
		this.leftDriveStick = leftDriveStick;
		this.rightDriveStick = rightDriveStick;
		driverGamepadGyroResetButton = new JoystickButton(driveController, RobotMap.START_BUTTON_VALUE);
		driverGamepadTuningActivation = new JoystickButton(driveController, RobotMap.BACK_BUTTON_VALUE);
		joystickRobotFrontCargoButton = new JoystickButton(leftDriveStick, 6);
		joystickRobotFrontHatchButton = new JoystickButton(leftDriveStick, 4);
		joystickGyroResetButton = new JoystickButton(rightDriveStick, 7);
    }

    public void runSwerve() {
		//this MUST be the first step so automation overrides everything else

		int driveMode = 0;

		double gamepadRobotTranslateX = squareWithSignReturn(driveController.getRawAxis(RobotMap.LEFT_STICK_X_AXIS));
		double gamepadRobotTranslateY = squareWithSignReturn(-driveController.getRawAxis(RobotMap.LEFT_STICK_Y_AXIS));
		double gamepadFieldTranslateX = squareWithSignReturn(driveController.getRawAxis(RobotMap.RIGHT_STICK_X_AXIS));
		double gamepadFieldTranslateY = squareWithSignReturn(-driveController.getRawAxis(RobotMap.RIGHT_STICK_Y_AXIS));
		double gamepadAbsoluteDirection = driveController.getPOV();
		double gamepadUnregTurning = squareWithSignReturn(driveController.getRawAxis(RobotMap.RIGHT_TRIGGER_AXIS) - driveController.getRawAxis(RobotMap.LEFT_TRIGGER_AXIS));

		double joystickRobotTranslateX = squareWithSignReturn(0);
		double joystickRobotTranslateY = squareWithSignReturn(0);
		double joystickFieldTranslateX = squareWithSignReturn(leftDriveStick.getX());
		double joystickFieldTranslateY = squareWithSignReturn(-leftDriveStick.getY());
		double joystickAbsoluteDirection = rightDriveStick.getDirectionDegrees();
		double joystickUnregTurning = squareWithSignReturn(rightDriveStick.getTwist());

		if (joystickAbsoluteDirection < 0) joystickAbsoluteDirection += 360;

		double robotTranslateX;
		double robotTranslateY;
		double fieldTranslateX;
		double fieldTranslateY;
		double absoluteDirection;
		double unregulatedTurning;

		if (rightDriveStick.getRawAxis(3) > 0) {
		    robotTranslateX = joystickRobotTranslateX;
			robotTranslateY = joystickRobotTranslateY;
			fieldTranslateX = joystickFieldTranslateX;
			fieldTranslateY = joystickFieldTranslateY;
			absoluteDirection = joystickAbsoluteDirection;
			unregulatedTurning = joystickUnregTurning;
			SmartDashboard.putBoolean("JoystickDriveEnabled", true);
		} else {
			robotTranslateX = gamepadRobotTranslateX;
			robotTranslateY = gamepadRobotTranslateY;
			fieldTranslateX = gamepadFieldTranslateX;
			fieldTranslateY = gamepadFieldTranslateY;
			absoluteDirection = gamepadAbsoluteDirection;
			unregulatedTurning = gamepadUnregTurning;
			SmartDashboard.putBoolean("JoystickDriveEnabled", false);
		}

		if(driveController.getYButton() || joystickRobotFrontCargoButton.get()) {
			setRobotFrontToCargo();
		} else if(driveController.getBButton() || joystickRobotFrontHatchButton.get()) {
			setRobotFrontToHatch();
		}
		if (driverGamepadTuningActivation.get() == true) {
			driveMode = 1;
		} else {
			driveMode = 0;
		} 

		switch (driveMode) {
			case 0:
				//the 0s are temporary replacements for the robot relative joysticks. remember to find the opposite of the y value
				translateAndRotate(fieldTranslateX, fieldTranslateY, unregulatedTurning, ahrs.getAngle() - ahrsOffset, absoluteDirection, robotTranslateX, robotTranslateY);
				break;

			case 1:
				tuningMode();
				break;

			case 2:

				break;
		
			default:
				break;
		}
		
		if (driverGamepadGyroResetButton.get() || joystickGyroResetButton.get()) {
			super.ahrsOffset = ahrs.getAngle();
			super.driveStraight = false;
			super.pidDrivingStraight.reset();
		}
			  
        SmartDashboard.putNumber("ahrs angle", ahrs.getAngle() - ahrsOffset);
       // SmartDashboard.putNumber("POV", driveController.getPOV());
		
	}
}