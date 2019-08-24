package frc.robot;

import java.awt.Robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDrive {

	enum selectiveSwerveDriveModes {
		ROBOT_UNREGULATED, ROBOT_ABSOLUTE, FIELD_UNREGULATED, FIELD_ABSOLUTE
	}

	AHRS ahrs;
	double ahrsOffset;
	PID pidDrivingStraight;
	RotatingBuffer gyroRateBuffer;
	RotatingBuffer[] modulePowerInput = new RotatingBuffer[4];
	RotatingBuffer[] encoderRateResponse = new RotatingBuffer[4];
	boolean driveStraight = false;
	double translationAngle;
	boolean isCargoFront = true;
	private selectiveSwerveDriveModes selectiveSwerveDriveMode;

	Timer swerveTimer = new Timer();
	double lastTimeStamp = swerveTimer.get();
	double currentTimeStamp = swerveTimer.get();
	double[] lastAcceleration = new double[4];
	double[] lastVelocity = new double[4];
	double[] currentAcceleration = new double[4];
	double[] currentJerk = new double[4];

	public boolean frSpeedEncoderIsWorking = true;
	public boolean flSpeedEncoderIsWorking = true;
	public boolean blSpeedEncoderIsWorking = true;
	public boolean brSpeedEncoderIsWorking = true;

	private static SwerveDrive swerveDrive;

	static SwerveModule frontRight = new SwerveModule(new TalonSRX(RobotMap.DRIVE_FRONT_RIGHT_MOTOR),
			new VictorSPX(RobotMap.STEER_FRONT_RIGHT_MOTOR), new AbsoluteAnalogEncoder(RobotMap.ENCODER_FRONT_RIGHT),
			RobotMap.ENCODER_ZERO_VALUE_FRONT_RIGHT, new Encoder(RobotMap.DRIVE_ENCODER_FRONT_RIGHT_A,
					RobotMap.DRIVE_ENCODER_FRONT_RIGHT_B, false, Encoder.EncodingType.k2X),
			"FrontRight");
	static SwerveModule frontLeft = new SwerveModule(new TalonSRX(RobotMap.DRIVE_FRONT_LEFT_MOTOR),
			new VictorSPX(RobotMap.STEER_FRONT_LEFT_MOTOR), new AbsoluteAnalogEncoder(RobotMap.ENCODER_FRONT_LEFT),
			RobotMap.ENCODER_ZERO_VALUE_FRONT_LEFT, new Encoder(RobotMap.DRIVE_ENCODER_FRONT_LEFT_A,
					RobotMap.DRIVE_ENCODER_FRONT_LEFT_B, false, Encoder.EncodingType.k2X),
			"FrontLeft");
	static SwerveModule backLeft = new SwerveModule(new TalonSRX(RobotMap.DRIVE_BACK_LEFT_MOTOR),
			new VictorSPX(RobotMap.STEER_BACK_LEFT_MOTOR), new AbsoluteAnalogEncoder(RobotMap.ENCODER_BACK_LEFT),
			RobotMap.ENCODER_ZERO_VALUE_BACK_LEFT, new Encoder(RobotMap.DRIVE_ENCODER_BACK_LEFT_A,
					RobotMap.DRIVE_ENCODER_BACK_LEFT_B, false, Encoder.EncodingType.k2X),
			"BackLeft");
	static SwerveModule backRight = new SwerveModule(new TalonSRX(RobotMap.DRIVE_BACK_RIGHT_MOTOR),
			new VictorSPX(RobotMap.STEER_BACK_RIGHT_MOTOR), new AbsoluteAnalogEncoder(RobotMap.ENCODER_BACK_RIGHT),
			RobotMap.ENCODER_ZERO_VALUE_BACK_RIGHT, new Encoder(RobotMap.DRIVE_ENCODER_BACK_RIGHT_A,
					RobotMap.DRIVE_ENCODER_BACK_RIGHT_B, false, Encoder.EncodingType.k2X),
			"BackRight");

	protected SwerveDrive(AHRS ahrs) {
		this.ahrs = ahrs;
		ahrsOffset = ahrs.getAngle();
		translationAngle = 0;

		pidDrivingStraight = new PID(0.0025, 0.000025, 0);
		pidDrivingStraight.setOutputRange(-0.5, 0.5);

		reset();
		gyroRateBuffer = new RotatingBuffer(5);
		for (int i = 0 ; i < 4 ; i++) {
			modulePowerInput[i] = new RotatingBuffer(5);
			encoderRateResponse[i] = new RotatingBuffer(5, 5);
			lastAcceleration[i] = 0;
			lastVelocity[i] = 0;
			currentAcceleration[i] = 0;
		}
	}

	public static SwerveDrive getInstance(AHRS ahrs) {
		if (swerveDrive == null) {
			swerveDrive = new SwerveDrive(ahrs);
		}
		return swerveDrive;
	}

	void selectiveTranslateAndRotate(selectiveSwerveDriveModes selectiveSwerveDriveMode, double turnInput,
			double translateXInput, double translateYInput) {
		this.selectiveSwerveDriveMode = selectiveSwerveDriveMode;
		switch (this.selectiveSwerveDriveMode) {
		case ROBOT_UNREGULATED:
			translateAndRotate(0, 0, turnInput, -1, translateXInput, translateYInput);
			break;
		case ROBOT_ABSOLUTE:
			translateAndRotate(0, 0, 0, turnInput, translateXInput, translateYInput);
			break;
		case FIELD_UNREGULATED:
			translateAndRotate(translateXInput, translateYInput, turnInput, -1, 0, 0);
			break;
		case FIELD_ABSOLUTE:
			translateAndRotate(translateXInput, translateYInput, 0, turnInput, 0, 0);
			break;
		}
	}

	void setAHRSOffset(double ahrsOffset) {
		this.ahrsOffset = ahrsOffset;
	}

	void syncroDrive(double driveSpeed, double driveAngle, double twist, double gyroReading) {
		// not field relative yet -- sitll needs work
		driveAngle += gyroReading;

		if (Math.abs(twist) > 0.5) {
			if (twist > 0) {
				twist = (twist - 0.5) * 2;
			} else if (twist < 0) {
				twist = (twist + 0.5) * 2;
			}
			frontRight.control(-twist, 45);
			frontLeft.control(twist, 315);
			backRight.control(-twist, 315);
			backLeft.control(twist, 45);
		} else {
			frontRight.control(driveSpeed, driveAngle);
			frontLeft.control(driveSpeed, driveAngle);
			backRight.control(driveSpeed, driveAngle);
			backLeft.control(driveSpeed, driveAngle);
		}

		SmartDashboard.putNumber("FR raw angle", frontRight.getAngle());
		SmartDashboard.putNumber("FL raw angle", frontLeft.getAngle());
		SmartDashboard.putNumber("BL raw angle", backLeft.getAngle());
		SmartDashboard.putNumber("BR raw angle", backRight.getAngle());
	}

	void tuningMode() {
		// used to tune the modules and their zero values

		SmartDashboard.putNumber("FR raw angle", frontRight.getAngle());
		SmartDashboard.putNumber("FL raw angle", frontLeft.getAngle());
		SmartDashboard.putNumber("BL raw angle", backLeft.getAngle());
		SmartDashboard.putNumber("BR raw angle", backRight.getAngle());
	}

	void setRobotFrontToCargo() {
		isCargoFront = true;
	}

	void setRobotFrontToHatch() {
		isCargoFront = false;
	}

	double squareWithSignReturn(double inputReading) {
		return Math.signum(inputReading) * inputReading * inputReading;
	}

	protected void reset() {
		frontRight.resetModule();
		frontLeft.resetModule();
		backLeft.resetModule();
		backRight.resetModule();
		frontLeft.invertModule();
		backLeft.invertModule();
	}

	// For testing purposes
	void individualModuleControl() {
		frontRight.setDriveSpeed(0);
		frontLeft.setDriveSpeed(0);
		backLeft.setDriveSpeed(0);
		backRight.setDriveSpeed(0);
		frontRight.setSteerSpeed(0);
		frontLeft.setSteerSpeed(0);
		backLeft.setSteerSpeed(0);
		backRight.setSteerSpeed(0);

		switch ((int) (SmartDashboard.getNumber("IMC", 0))) {
		case 0:
			frontRight.setDriveSpeed(0.3);
			break;
		case 1:
			frontLeft.setDriveSpeed(0.3);
			break;
		case 2:
			backLeft.setDriveSpeed(0.3);
			break;
		case 3:
			backRight.setDriveSpeed(0.3);
			break;
		case 4:
			frontRight.setSteerSpeed(0.3);
			break;
		case 5:
			frontLeft.setSteerSpeed(0.3);
			break;
		case 6:
			backLeft.setSteerSpeed(0.3);
			break;
		case 7:
			backRight.setSteerSpeed(0.3);
			break;
		default:
			break;
		}

		SmartDashboard.putNumber("FR raw angle", frontRight.getAngle());
		SmartDashboard.putNumber("FL raw angle", frontLeft.getAngle());
		SmartDashboard.putNumber("BL raw angle", backLeft.getAngle());
		SmartDashboard.putNumber("BR raw angle", backRight.getAngle());

		// SmartDashboard.putNumber("Corrected angle FR",
		// frontRight.convertToRobotRelative(frontRight.getAngle()));
		// SmartDashboard.putNumber("Corrected angle FL",
		// frontLeft.convertToRobotRelative(frontLeft.getAngle()));
		// SmartDashboard.putNumber("Corrected angle BR",
		// backRight.convertToRobotRelative(backRight.getAngle()));
		// SmartDashboard.putNumber("Corrected angle BL",
		// backLeft.convertToRobotRelative(backLeft.getAngle()));
	}

	void testSwerveModule(boolean isFront, boolean isLeft, double driveSpeed, double steerSpeed) {
		if (isFront && isLeft) {
			frontLeft.setDriveSpeed(driveSpeed);
			frontLeft.setSteerSpeed(steerSpeed);
		} else if (isFront && !isLeft) {
			frontRight.setDriveSpeed(driveSpeed);
			frontRight.setSteerSpeed(steerSpeed);
		} else if (!isFront && isLeft) {
			backLeft.setDriveSpeed(driveSpeed);
			backLeft.setSteerSpeed(steerSpeed);
		} else if (!isFront && !isLeft) {
			backRight.setDriveSpeed(driveSpeed);
			backRight.setSteerSpeed(steerSpeed);
		}
	}

	
	void translateAndRotateCM(double driveFieldTranslationX, double driveFieldTranslationY, double unregulatedTurning, double fieldRelativeRobotDirection, double driveRobotTranslationX, double driveRobotTranslationY) {
		translateAndRotate(driveFieldTranslationX / RobotMap.MAX_CM_PER_SECOND, driveFieldTranslationY / RobotMap.MAX_CM_PER_SECOND, unregulatedTurning, fieldRelativeRobotDirection, driveRobotTranslationX / RobotMap.MAX_CM_PER_SECOND, driveRobotTranslationY / RobotMap.MAX_CM_PER_SECOND);
	}

	void translateAndRotate(double driveFieldTranslationX, double driveFieldTranslationY, double unregulatedTurning, double fieldRelativeRobotDirection, double driveRobotTranslationX,
			double driveRobotTranslationY) {
		// turns the gyro into a 0-360 range -- easier to work with
		double gyroValueUnprocessed = ahrs.getAngle() - this.ahrsOffset;
		double gyroValueProcessed = (Math.abs(((int) (gyroValueUnprocessed)) * 360) + gyroValueUnprocessed) % 360;

		// initializing the main variables
		Point fieldRelativeVector = new Point(driveFieldTranslationX, driveFieldTranslationY);
		Point robotRelativeVector = new Point(driveRobotTranslationX, driveRobotTranslationY);
		double unregulatedRotationValue = unregulatedTurning;
		double absoluteFieldRelativeDirection = fieldRelativeRobotDirection;

		//merges/chooses the two translation vectors
		Point translationVector = convertToFieldRelative(robotRelativeVector, fieldRelativeVector, gyroValueProcessed);
		
		//compressing turn based parameters into a single vector
		Point rotationVector = rotationMagnitude(translationVector, absoluteFieldRelativeDirection, unregulatedRotationValue, gyroValueProcessed, gyroValueUnprocessed);

		//combining translation and rotation vectors into four vectors, one per wheel
		Point[] wheelVector = new Point[4];

		wheelVector[0] = GeometricMath.vectorAddition(translationVector, GeometricMath.rotateVector(rotationVector, 90));
		wheelVector[1] = GeometricMath.vectorAddition(translationVector, GeometricMath.rotateVector(rotationVector, 0));
		wheelVector[2] = GeometricMath.vectorAddition(translationVector, GeometricMath.rotateVector(rotationVector, 270));
		wheelVector[3] = GeometricMath.vectorAddition(translationVector, GeometricMath.rotateVector(rotationVector, 180));

		wheelVector = speedScale(wheelVector, 1);

		//wheelSpeed = limitForces(wheelSpeed);
		//updateModuleHardwareStates(wheelVector);
		setModules(wheelVector);

		//dataShoot(gyroValueProcessed);
	}

	double[] limitForces(double[] currentVelocity) {
		currentTimeStamp = swerveTimer.get();
		for (int i = 0 ; i < 4 ; i ++) {
			currentAcceleration[i] = (currentVelocity[i] - lastVelocity[i]) / (currentTimeStamp - lastTimeStamp);
			if (Math.abs(currentAcceleration[i]) > RobotMap.DRIVE_MAX_ACCELERATION_PER_SECOND) {
				if (currentAcceleration[i] > 0) {
					currentVelocity[i] = lastVelocity[i] + ((currentTimeStamp - lastTimeStamp) * RobotMap.DRIVE_MAX_ACCELERATION_PER_SECOND);
				} else {
					currentVelocity[i] = lastVelocity[i] - ((currentTimeStamp - lastTimeStamp) * RobotMap.DRIVE_MAX_ACCELERATION_PER_SECOND);
				}
				currentAcceleration[i] = (currentVelocity[i] - lastVelocity[i]) / (currentTimeStamp - lastTimeStamp);
			}
			currentJerk[i] = (currentAcceleration[i] - lastAcceleration[i]) / (currentTimeStamp - lastTimeStamp);
			if (Math.abs(currentJerk[i]) > RobotMap.DRIVE_MAX_JERK_PER_SECOND) {
				//MATH NEEDS TO BE FIXED
				if (currentVelocity[i] > 0) {
					if (currentJerk[i] > 0) {
						currentVelocity[i] = lastVelocity[i] + ((currentTimeStamp - lastTimeStamp) * (lastAcceleration[i] + ((currentTimeStamp - lastTimeStamp) * RobotMap.DRIVE_MAX_JERK_PER_SECOND)));
					} else {
						currentVelocity[i] = lastVelocity[i] - ((currentTimeStamp - lastTimeStamp) * (lastAcceleration[i] + ((currentTimeStamp - lastTimeStamp) * RobotMap.DRIVE_MAX_JERK_PER_SECOND)));
					}
				} else {
					if (currentJerk[i] > 0) {
						currentVelocity[i] = lastVelocity[i] + ((currentTimeStamp - lastTimeStamp) * (lastAcceleration[i] - ((currentTimeStamp - lastTimeStamp) * RobotMap.DRIVE_MAX_JERK_PER_SECOND)));
					} else {
						currentVelocity[i] = lastVelocity[i] - ((currentTimeStamp - lastTimeStamp) * (lastAcceleration[i] - ((currentTimeStamp - lastTimeStamp) * RobotMap.DRIVE_MAX_JERK_PER_SECOND)));
					}
				}
				currentAcceleration[i] = (currentVelocity[i] - lastVelocity[i]) / (currentTimeStamp - lastTimeStamp);
			}
			

			lastVelocity[i] = currentVelocity[i];
			lastAcceleration[i] = currentAcceleration[i];
		}
		return currentVelocity;
	}

	void updateModuleHardwareStates(Point[] wheelSpeeds) {

		modulePowerInput[0].add(wheelSpeeds[0].distanceFromZero());
		encoderRateResponse[0].add(frontRightDegreesPerSecond());
		modulePowerInput[1].add(wheelSpeeds[1].distanceFromZero());
		encoderRateResponse[1].add(frontLeftDegreesPerSecond());
		modulePowerInput[2].add(wheelSpeeds[2].distanceFromZero());
		encoderRateResponse[2].add(backLeftDegreesPerSecond());
		modulePowerInput[3].add(wheelSpeeds[3].distanceFromZero());
		encoderRateResponse[3].add(backRightDegreesPerSecond());

		if (modulePowerInput[0].getAverage() > 0.8 && encoderRateResponse[0].getAverage() < 10) {
			frSpeedEncoderIsWorking = false;
		} else if (modulePowerInput[0].getAverage() > 0.8 && encoderRateResponse[0].getAverage() > 20) {
			frSpeedEncoderIsWorking = true;
		}

		if (modulePowerInput[1].getAverage() > 0.8 && encoderRateResponse[1].getAverage() < 10) {
			flSpeedEncoderIsWorking = false;
		} else if (modulePowerInput[1].getAverage() > 0.8 && encoderRateResponse[1].getAverage() > 20) {
			flSpeedEncoderIsWorking = true;
		}

		if (modulePowerInput[2].getAverage() > 0.8 && encoderRateResponse[2].getAverage() < 10) {
			blSpeedEncoderIsWorking = false;
		} else if (modulePowerInput[2].getAverage() > 0.8 && encoderRateResponse[2].getAverage() > 20) {
			blSpeedEncoderIsWorking = true;
		}
		
		if (modulePowerInput[3].getAverage() > 0.8 && encoderRateResponse[3].getAverage() < 10) {
			brSpeedEncoderIsWorking = false;
		} else if (modulePowerInput[3].getAverage() > 0.8 && encoderRateResponse[3].getAverage() > 20) {
			brSpeedEncoderIsWorking = true;
		}

		frontRight.driveEncoderWorking = frSpeedEncoderIsWorking;
		frontLeft.driveEncoderWorking = flSpeedEncoderIsWorking;
		backLeft.driveEncoderWorking = blSpeedEncoderIsWorking;
		backRight.driveEncoderWorking = brSpeedEncoderIsWorking;

	}

	Point convertToFieldRelative(Point robotRelativeVector, Point fieldRelativeVector, double drivetrainAngle) {
		double fieldTransMag = fieldRelativeVector.distanceFromZero();
		if (fieldTransMag != 0) {
			double initialAngle;
			if (fieldRelativeVector.getX() == 0) {
				if (fieldRelativeVector.getY() > 0) {
					initialAngle = 90;
				} else {
					initialAngle = -90;
				}
			} else {
				initialAngle = Math.toDegrees(Math.atan(fieldRelativeVector.getY() / fieldRelativeVector.getX()));
			}
			if (fieldRelativeVector.getX() < 0) {
				if (fieldRelativeVector.getY()> 0) {
					initialAngle += 180;
				} else {
					initialAngle -= 180;
				}
			}
			double processedAngle = initialAngle + drivetrainAngle;
			robotRelativeVector.setX(fieldTransMag * Math.cos(Math.toRadians(processedAngle)));
			robotRelativeVector.setY(fieldTransMag * Math.sin(Math.toRadians(processedAngle)));
		}
		return robotRelativeVector;
	}

	Point rotationMagnitude(Point translation, double absoluteTargetAngle, double unregulatedTurnValue, double drivetrainCompassHeading, double drivetrainAngleAccumulated) {
		gyroRateBuffer.add(ahrs.getRate());
		double rotationMagnitude;
		if (absoluteTargetAngle != -1) {
			if (absoluteTargetAngle < 0)
				absoluteTargetAngle += 360;
			if (drivetrainCompassHeading > 180 && absoluteTargetAngle == 0) {
				absoluteTargetAngle = 360;
			}
			rotationMagnitude = (absoluteTargetAngle - drivetrainCompassHeading) * 0.005;
			if (Math.abs(absoluteTargetAngle - drivetrainCompassHeading) > 180)
				rotationMagnitude *= -1;
			driveStraight = false;
		} else if (unregulatedTurnValue > RobotMap.ROTATION_DEADZONE
				|| unregulatedTurnValue < -RobotMap.ROTATION_DEADZONE) {
			rotationMagnitude = unregulatedTurnValue;
			driveStraight = false;
		} else if (Math.sqrt(Math.pow(translation.getX(), 2) + Math.pow(translation.getY(), 2)) > RobotMap.TRANSLATION_DEADZONE * 0.75 && Math.abs(gyroRateBuffer.getAverage()) < 3) {
			// no turning methods -- goes straight
			if (driveStraight == false) {
				driveStraight = true;
				translationAngle = drivetrainAngleAccumulated;
				pidDrivingStraight.reset();
			}
			pidDrivingStraight.setError(drivetrainAngleAccumulated - translationAngle);
			pidDrivingStraight.update();
			rotationMagnitude = -pidDrivingStraight.getOutput();
		} else {
			rotationMagnitude = 0;
		}
		if (rotationMagnitude > 1) {
			rotationMagnitude = 1;
		} else if (rotationMagnitude < -1) {
			rotationMagnitude = -1;
		}
		return new Point(rotationMagnitude, rotationMagnitude);
	}

	Point[] speedScale(Point[] speedSet, double speedLimit) {
		double maxSpeed = speedSet[0].distanceFromZero();
		if (speedSet[1].distanceFromZero() > maxSpeed) {
			maxSpeed = speedSet[1].distanceFromZero();
		}
		if (speedSet[2].distanceFromZero() > maxSpeed) {
			maxSpeed = speedSet[2].distanceFromZero();
		}
		if (speedSet[3].distanceFromZero() > maxSpeed) {
			maxSpeed = speedSet[3].distanceFromZero();
		}
		if (maxSpeed > speedLimit) {
			for (int i = 0; i < 4; i++) {
				speedSet[i] = GeometricMath.scaleVector(speedSet[i], 1 / maxSpeed);
			}
		}
		return speedSet;
	}

	void setModules(Point[] vectors) {
		frontRight.control(vectors[0].distanceFromZero(), 360 - vectors[0].getCompassAngle());
		frontLeft.control(vectors[1].distanceFromZero(), 360 - vectors[1].getCompassAngle());
		backLeft.control(vectors[2].distanceFromZero(), 360 - vectors[2].getCompassAngle());
		backRight.control(vectors[3].distanceFromZero(), 360 - vectors[3].getCompassAngle());
		SmartDashboard.putNumber("Data1", vectors[0].distanceFromZero());
		// double power = 0.42342342934797;
		// frontRight.control(power, 0);
		// SmartDashboard.putNumber("translationVector", power);
		// SmartDashboard.putNumber("data1", vectors[0].distanceFromZero());
		// frontRight.control(0, 360 - vectors[0].getCompassAngle());
		// frontLeft.control(0, 360 - vectors[1].getCompassAngle());
		// backLeft.control(0, 360 - vectors[2].getCompassAngle());
		// backRight.control(0, 360 - vectors[3].getCompassAngle());
	}


	void dataShoot(double gyroValue) {
		// reads out the raw angles, processed angles, speed, and gyro
		SmartDashboard.putNumber("FR raw angle", frontRight.getAngle());
		SmartDashboard.putNumber("FL raw angle", frontLeft.getAngle());
		SmartDashboard.putNumber("BL raw angle", backLeft.getAngle());
		SmartDashboard.putNumber("BR raw angle", backRight.getAngle());

		SmartDashboard.putNumber("Gyro 0-360", gyroValue);

		// SmartDashboard.putNumber("FR Speed", wheelSpeed[0]);
		// SmartDashboard.putNumber("FL Speed", wheelSpeed[1]);
		// SmartDashboard.putNumber("BL Speed", wheelSpeed[2]);
		// SmartDashboard.putNumber("BR Speed", wheelSpeed[3]);

		// SmartDashboard.putNumber("FR Angle", wheelAngle[0]);
		// SmartDashboard.putNumber("FL Angle", wheelAngle[1]);
		// SmartDashboard.putNumber("BL Angle", wheelAngle[2]);
		// SmartDashboard.putNumber("BR Angle", wheelAngle[3]);
	}

	public double frontRightDegreesPerSecond() {
		return frontRight.getRate();
	}
	public double frontLeftDegreesPerSecond() {
		return frontLeft.getRate();
	}
	public double backLeftDegreesPerSecond() {
		return backLeft.getRate();
	}
	public double backRightDegreesPerSecond() {
		return backRight.getRate();
	}

	public double frontRightAngle() {
		return frontRight.getAngle();
	}
	public double frontLeftAngle() {
		return frontLeft.getAngle();
	}
	public double backLeftAngle() {
		return backLeft.getAngle();
	}
	public double backRightAngle() {
		return backRight.getAngle();
	}

	public double frontRightCMPerSecond() {
		return frontRightDegreesPerSecond() * RobotMap.INCH_TO_CM * RobotMap.SWERVE_WHEEL_DIAMETER * Math.PI / 360;
	}
	public double frontLeftCMPerSecond() {
		return frontLeftDegreesPerSecond() * 2.54 * 6 * 3.14 / 360;
	}
	public double backLeftCMPerSecond() {
		return backLeftDegreesPerSecond() * 2.54 * 6 * 3.14 / 360;
	}
	public double backRightCMPerSecond() {
		return backRightDegreesPerSecond() * 2.54 * 6 * 3.14 / 360;
	}

}