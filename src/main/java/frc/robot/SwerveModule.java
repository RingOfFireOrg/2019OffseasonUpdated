package frc.robot;

import java.awt.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.PIDController;

//need to ifx
public class SwerveModule {
	TalonSRX drive;
	VictorSPX steer;
	AbsoluteAnalogEncoder turnEncoder;
	Encoder driveEncoder;
	double speed;
	double turnSpeed;
	double angleGoal;
	double currentAngle;
	double zeroValue;
	String moduleName;
	double optimizedSpeed;
	PID speedRegulation;
	PIDConstants swerveDriveSpeedGains = new PIDConstants(1, -0.000001, 0);
	PID swerveSteer;
	//TODO tune swerve steer PID
	PIDConstants swerveSteerGains = new PIDConstants(0.01, 0, 0);
	double accumulatedGR = 0;
	int powerInversion = 1;
	static final double MAX_STEER_POWER = 0.8;
	public boolean driveEncoderWorking = true; //try switching this to false for initiative -- CM (8-15-19)
	

	//will need to make changes to the input --Encoder driveRotEncoder <-- add to constructor
	public SwerveModule(TalonSRX driveMotor, VictorSPX steerMotor, AbsoluteAnalogEncoder steerEncoder, double zeroValue, Encoder driveRotEncoder, String name) {
		this.zeroValue = zeroValue;
		drive = driveMotor;
		steer = steerMotor;
		turnEncoder = steerEncoder;
		moduleName = name;
		driveEncoder = driveRotEncoder;

		driveEncoder.reset();
		driveEncoder.setDistancePerPulse(18); //in degrees (360)/(20 pulses per rotation)

		speedRegulation = new PID(swerveDriveSpeedGains.getKP(), swerveDriveSpeedGains.getKI(), swerveDriveSpeedGains.getKD());
		speedRegulation.setOutputRange(-RobotMap.MAX_DRIVE_POWER, RobotMap.MAX_DRIVE_POWER);
		speedRegulation.reset();

		swerveSteer = new PID(swerveSteerGains.getKP(), swerveSteerGains.getKI(), swerveSteerGains.getKD());
		swerveSteer.setOutputRange(-1, 1);
		swerveSteer.reset();
	}

	public void invertModule() {
		powerInversion = -1;
	}

	public void resetModule() {
		speedRegulation.reset();
	}

	public double getRawAngle() {
		return turnEncoder.getAngle();
	}

	public double getRate() {
		return driveEncoder.getRate(); //in degrees per second
	}

	public void stop() {
		drive.set(ControlMode.PercentOutput, 0);
		steer.set(ControlMode.PercentOutput, 0);
	}

	public void setDriveSpeed(double drivePower) {
		drivePower *= powerInversion;
		if (drivePower > RobotMap.MAX_DRIVE_POWER) {
			drivePower = RobotMap.MAX_DRIVE_POWER;
		} else if (drivePower < -RobotMap.MAX_DRIVE_POWER) {
			drivePower = -RobotMap.MAX_DRIVE_POWER;
		}
		// if (driveEncoderWorking) {
		// 	speedRegulation.setError(drivePower - ((getRate() * RobotMap.DPS_TO_RPM) / (RobotMap.MAX_SWERVE_SPEED_IN_RPM * RobotMap.DRIVE_GEARING_RATIO)) );
		// 	speedRegulation.update();
		// 	optimizedSpeed = GeometricMath.limitRange(drivePower + speedRegulation.getOutput(), -RobotMap.MAX_DRIVE_POWER, RobotMap.MAX_DRIVE_POWER);
		// 	drive.set(ControlMode.PercentOutput, optimizedSpeed);
		// } else {
			drive.set(ControlMode.PercentOutput, drivePower);
			SmartDashboard.putNumber("Data2", drivePower);
		//}
		
		
		//SmartDashboard.putNumber("OS - " + moduleName, optimizedSpeed);
		//SmartDashboard.putNumber("DP - " + moduleName, drivePower);
		//SmartDashboard.putNumber("SR - " + moduleName, speedRegulation.getOutput());
		//SmartDashboard.putNumber("GR - " + moduleName, accumulatedGR);
		
	}

	public void setSteerSpeed(double steerPower) {

		if (steerPower > MAX_STEER_POWER) {
			steerPower = MAX_STEER_POWER;
		} else if (steerPower < -MAX_STEER_POWER) {
			steerPower = -MAX_STEER_POWER;
		}
		steer.set(ControlMode.PercentOutput, steerPower);
		
	}
	
	public double steerAngleDeviation(double goalRobotRelativeAngle) {
		double error = (((360 - goalRobotRelativeAngle) - robotRelativeAngle()) + 720) % 360;
		if (error > 180) error -= 360;
		return error;
		//will return an error between -180 & 180
	}

	public void control(double goalDriveSpeed, double goalSteerAngle) {
		//Complete rebuild -- simplified -- untested 9/7/19 CM
		SmartDashboard.putNumber("WheelSpeed:D/S-" + moduleName, getRate());
		double steerAngularError = steerAngleDeviation(goalSteerAngle);

		if (Math.abs(steerAngularError) > 90) {
			if (steerAngularError > 0) {
				steerAngularError -= 180;
			} else {
				steerAngularError += 180;
			}
			goalDriveSpeed *= -1;
		}
		swerveSteer.setError(steerAngularError);
		swerveSteer.update();
		setSteerSpeed(swerveSteer.getOutput());
		setDriveSpeed(goalDriveSpeed);
	}

	public double driveEncoderDegrees() {
		return driveEncoder.get();
	}

	public double driveEncoderDegrees(double relative) {
		return driveEncoder.get() - relative;
	}

	public double convertToWheelRelative(double angle) {
		return ((angle + zeroValue) + 720) % 360;
	}

	public double convertToRobotRelative(double angle) {
		return ((angle - zeroValue) + 720) % 360;
	}
	
	public double robotRelativeAngle() {
		return convertToRobotRelative(getRawAngle());
	}
}
