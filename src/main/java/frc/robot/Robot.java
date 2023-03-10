// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  Joystick joy = new Joystick(0);
  //XboxController xbox = new XboxController(1);

  //arcade drive
  // CANSparkMax leftMotor1 = new CANSparkMax(0, MotorType.kBrushed);
  // CANSparkMax leftMotor2 = new CANSparkMax(1, MotorType.kBrushed);
  // CANSparkMax rightMotor1 = new CANSparkMax(2, MotorType.kBrushed);
  // CANSparkMax rightMotor2 = new CANSparkMax(3, MotorType.kBrushed);

  // MotorControllerGroup leftMotors = new MotorControllerGroup(leftMotor1, leftMotor2);
  // MotorControllerGroup rightMotors = new MotorControllerGroup(rightMotor1, rightMotor2);

  VictorSP leftMotor1 = new VictorSP(0);
  VictorSP leftMotor2 = new VictorSP(1);
  VictorSP leftMotor3 = new VictorSP(2);

  VictorSP rightMotor1 = new VictorSP(3);
  VictorSP rightMotor2 = new VictorSP(4);
  VictorSP rightMotor3 = new VictorSP(5);

  MotorControllerGroup leftMotors = new MotorControllerGroup(leftMotor1, leftMotor2, leftMotor3);
  MotorControllerGroup rightMotors = new MotorControllerGroup(rightMotor1, rightMotor2, rightMotor3);

  DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);

  //pid
  // Encoder encoder = new Encoder(0, 1);
  // PIDController pid = new PIDController(0.15, 0.01, 0);

  //pneumatics
  Compressor phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);
  DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    //encoder.reset();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // double currentPos = encoder.getDistance();
    // double speed = pid.calculate(currentPos * k_Tick2Feet, 3);
    // drive.arcadeDrive(speed, -speed);
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    drive.setSafetyEnabled(false);
    phCompressor.disable();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    drive.arcadeDrive(-joy.getRawAxis(1), joy.getRawAxis(0));

    if (joy.getRawButtonPressed(1)) {
      solenoid.toggle();
    }

    // if (joy.getRawButton(0)) {
    //   solenoid.set(Value.kForward);
    // } else if (joy.getRawButton(1)) {
    //   solenoid.set(Value.kReverse);
    // }

    if (joy.getRawButtonPressed(2)) {
      phCompressor.enableDigital();
    } else if (joy.getRawButtonPressed(3)) {
      phCompressor.disable();
    }

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

}

