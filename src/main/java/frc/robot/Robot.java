/*----------------------------------------------------------------------------*/
/* Destination Deep Space Robot - 2019 Team PyroTech (FRC 3459)               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Prototype_PWM;

/**
 * Don't change the name of this or it won't work. (The manifest looks for
 * "Robot")
 */
public class Robot extends TimedRobot {
  Prototype_PWM crossbowPwm;
  Joystick mainStick = new Joystick(RobotMap.JOYSTICK_MAIN);
  Compressor compressor = new Compressor(0);
  DoubleSolenoid grabber = new DoubleSolenoid (RobotMap.GRABBER_SOLENOID_BOTTOM,RobotMap.GRABBER_SOLENOID_TOP);

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    crossbowPwm = new Prototype_PWM(RobotMap.PWM_TEST_ATTACHMENT, RobotMap.SPEED_DEFAULT_TEST);
    compressor.setClosedLoopControl(true);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This function is called once we go into autonomous mode
   */
  @Override
  public void autonomousInit() {
  }

  /**
   * This function is called periodically during autonomous. (approx 20ms)
   */
  @Override
  public void autonomousPeriodic() {
  }

  /**
   * This function is called when you switch into teleop mode on the driver
   * station.
   */
  @Override
  public void teleopInit() {

  }

  /**
   * This function is called periodically during operator control. (approx 20ms)
   */
  @Override
  public void teleopPeriodic() {
    double yPos = mainStick.getY();

    // The 0.25 and -0.25 are so that the joystick doesn't have to be perfectly
    // centered to stop
    if (yPos > 0.25) {
      crossbowPwm.forward();
    } else if (yPos < -0.25) {
      crossbowPwm.reverse();
    } else {
      crossbowPwm.stop();
    }
    if(mainStick.getRawButton(RobotMap.BUTTON_GRABBER_CLOSE)){
      grabber.set(DoubleSolenoid.Value.kReverse); 
    }
    else if(mainStick.getRawButton(RobotMap.BUTTON_GRABBER_OPEN)){
      grabber.set(DoubleSolenoid.Value.kForward);
    }
    else  {
      grabber.set(DoubleSolenoid.Value.kOff);
    }

  }

  /**
   * This function is called when you switch into teleop mode on the driver
   * station.
   */
  @Override
  public void testInit() {
  }

  /**
   * This function is called periodically during test mode. (approx 20ms)
   */
  @Override
  public void testPeriodic() {
  }
}
