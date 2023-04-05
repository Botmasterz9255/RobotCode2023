// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;


public class Robot extends TimedRobot {
  /*
   * Autonomous selection options.
   */
  private static final String kNothingAuto = "do nothing";
  private static final String kConeAuto = "cone";
  private static final String kCubeAuto = "cube";
  private static final String kCubeEngageAuto = "cube engage";
  private static final String kConeEngageAuto = "cone engage";
  private String m_autoSelected;
  private AutoBalance m_autoBalance;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /*
   * Drive motor controller instances.
   * 
   * Change the id's to match your robot.
   * Change kBrushed to kBrushless if you are using NEO's.
   * Use the appropriate other class if you are using different controllers.
   */

  CANSparkMax driveLeftSpark = new CANSparkMax(2, MotorType.kBrushed);
  CANSparkMax driveRightSpark = new CANSparkMax(4, MotorType.kBrushed);
  WPI_VictorSPX driveLeftVictor = new WPI_VictorSPX(10);
  WPI_VictorSPX driveRightVictor = new WPI_VictorSPX(11);
  

  /*
   * Mechanism motor controller instances.
   * 
   * Like the drive motors, set the CAN id's to match your robot or use different
   * motor controller classses (TalonFX, TalonSRX, Spark, VictorSP) to match your
   * robot.
   * 
   * The arm is a NEO on Everybud.
   * The intake is a NEO 550 on Everybud.
   */
  CANSparkMax arm = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax intake = new CANSparkMax(5, MotorType.kBrushless);

  /**
   * The starter code uses the most generic joystick class.
   * 
   * The reveal video was filmed using a logitech gamepad set to
   * directinput mode (switch set to D on the bottom). You may want
   * to use the XBoxController class with the gamepad set to XInput
   * mode (switch set to X on the bottom) or a different controller
   * that you feel is more comfortable.
   */
  XboxController driverJoystick = new XboxController(0);
  XboxController armJoystick = new XboxController(1);
  
  /*
   * Magic numbers. Use these to adjust settings.
   * ADjust this up and down for over all robot speed
   */
  static final double POWER = 0.16;

  /**
   * How many amps the arm motor can use.
   */
  static final int ARM_CURRENT_LIMIT_A = 10;

  /**
   * Percent output to run the arm up/down at
   */
  static final double ARM_OUTPUT_POWER = 0.20;

  /**
   * How many amps the intake can use while picking up
   */
  static final int INTAKE_CURRENT_LIMIT_A = 25;

  /**
   * How many amps the intake can use while holding
   */
  static final int INTAKE_HOLD_CURRENT_LIMIT_A = 5;

  /**
   * Percent output for intaking
   */
  static final double INTAKE_OUTPUT_POWER = 0.75; //was 1

  /**
   * Percent output for holding
   */
  static final double INTAKE_HOLD_POWER = 0.07;

  /**
   * Time to extend or retract arm in auto
   */
  static final double ARM_EXTEND_TIME_S = 2.0;

  /**
   * Time to throw game piece in auto
   */
  static final double AUTO_THROW_TIME_S = 0.375;

  /**
   * Time to drive back in auto
   */
  static final double AUTO_DRIVE_TIME = 5.0;

  /**
   * Speed to drive backwards in auto
   */
  static final double AUTO_DRIVE_SPEED = -0.25;

  /**
   * This method is run once when the robot is first started up.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("do nothing", kNothingAuto);
    m_chooser.addOption("cone and mobility", kConeAuto);
    m_chooser.addOption("cube and mobility", kCubeAuto);
    m_chooser.addOption("cube and engage", kCubeEngageAuto);
    m_chooser.addOption("cone and engage", kConeEngageAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    CameraServer.startAutomaticCapture();

    
    m_autoBalance = new AutoBalance();

    /*
     * You will need to change some of these from false to true.
     * 
     * In the setDriveMotors method, comment out all but 1 of the 4 calls
     * to the set() methods. Push the joystick forward. Reverse the motor
     * if it is going the wrong way. Repeat for the other 3 motors.
     */
    driveLeftSpark.setInverted(true);
    driveLeftVictor.setInverted(true);
    driveRightSpark.setInverted(false);
    driveRightVictor.setInverted(false);

    /*
     * Set the arm and intake to brake mode to help hold position.
     * If either one is reversed, change that here too. Arm out is defined
     * as positive, arm in is negative.
     */
    arm.setInverted(true);
    arm.setIdleMode(IdleMode.kBrake);
    arm.setSmartCurrentLimit(ARM_CURRENT_LIMIT_A);
    intake.setInverted(false);
    intake.setIdleMode(IdleMode.kBrake);
  }

  /**
   * Calculate and set the power to apply to the left and right
   * drive motors.
   * 
   * @param forward Desired forward speed. Positive is forward.
   * @param turn    Desired turning speed. Positive is counter clockwise from
   *                above.
   */
  public void setDriveMotors(double forward, double turn) {
    SmartDashboard.putNumber("drive forward power (%)", forward);
    SmartDashboard.putNumber("drive turn power (%)", turn);

    /*
     * positive turn = counter clockwise, so the left side goes backwards
     */
    double left = forward - turn;
    double right = forward + turn;

    SmartDashboard.putNumber("drive left power (%)", left);
    SmartDashboard.putNumber("drive right power (%)", right);

    // see note above in robotInit about commenting these out one by one to set
    // directions.
    driveLeftSpark.set(left);
    driveLeftVictor.set(ControlMode.PercentOutput, left);
    driveRightSpark.set(right);
    driveRightVictor.set(ControlMode.PercentOutput, right);
  }

  /**
   * Set the arm output power. Positive is out, negative is in.
   * 
   * @param percent
   */
  public void setArmMotor(double percent) {
    arm.set(percent);
    SmartDashboard.putNumber("arm power (%)", percent);
    SmartDashboard.putNumber("arm motor current (amps)", arm.getOutputCurrent());
    SmartDashboard.putNumber("arm motor temperature (C)", arm.getMotorTemperature());
  }

  /**
   * Set the arm output power.
   * 
   * @param percent desired speed
   * @param amps current limit
   */
  public void setIntakeMotor(double percent, int amps) {
    intake.set(percent);
    intake.setSmartCurrentLimit(amps);
    SmartDashboard.putNumber("intake power (%)", percent);
    SmartDashboard.putNumber("intake motor current (amps)", intake.getOutputCurrent());
    SmartDashboard.putNumber("intake motor temperature (C)", intake.getMotorTemperature());
  }

  /**
   * This method is called every 20 ms, no matter the mode. It runs after
   * the autonomous and teleop specific period methods.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Time (seconds)", Timer.getFPGATimestamp());
  }

  double autonomousStartTime;
  double autonomousIntakePower;

  @Override
  public void autonomousInit() {
    driveLeftSpark.setIdleMode(IdleMode.kBrake);
    driveLeftVictor.setNeutralMode(NeutralMode.Brake);
    driveRightSpark.setIdleMode(IdleMode.kBrake);
    driveRightVictor.setNeutralMode(NeutralMode.Brake);

    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);

    if (m_autoSelected == kConeAuto || m_autoSelected == kConeEngageAuto) {
      autonomousIntakePower = INTAKE_OUTPUT_POWER;
    } else if (m_autoSelected == kCubeAuto || m_autoSelected == kCubeEngageAuto) {
      autonomousIntakePower = -INTAKE_OUTPUT_POWER;
    }

    autonomousStartTime = Timer.getFPGATimestamp();
  }

  @Override
  public void autonomousPeriodic() {

    double timeElapsed = Timer.getFPGATimestamp() - autonomousStartTime;

    if (m_autoSelected == kNothingAuto) {

      setArmMotor(0.0);
      setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(0.0, 0.0);

    } else if(m_autoSelected == kConeEngageAuto || m_autoSelected == kCubeEngageAuto) {

      if (timeElapsed < ARM_EXTEND_TIME_S) {
        setArmMotor(ARM_OUTPUT_POWER);
        setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
        setDriveMotors(0.0, 0.0);
      } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S) {
        setArmMotor(0.0);
        setIntakeMotor(autonomousIntakePower, INTAKE_CURRENT_LIMIT_A);
        setDriveMotors(0.0, 0.0);
      } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S + ARM_EXTEND_TIME_S) {
        setArmMotor(-ARM_OUTPUT_POWER);
        setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
        setDriveMotors(0.0, 0.0);
      } else if(timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S + ARM_EXTEND_TIME_S + 2.57) {
        SmartDashboard.putNumber("isDriving", 1);
        setArmMotor(0.0);
        setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
        setDriveMotors(AUTO_DRIVE_SPEED*2, 0.0);

      } else {
        setDriveMotors(0, 0.0);
      }

    } else if(m_autoSelected == kConeAuto || m_autoSelected == kCubeAuto) {

      if (timeElapsed < ARM_EXTEND_TIME_S) {
        setArmMotor(ARM_OUTPUT_POWER);
        setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
        setDriveMotors(0.0, 0.0);
      } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S) {
        setArmMotor(0.0);
        setIntakeMotor(autonomousIntakePower, INTAKE_CURRENT_LIMIT_A);
        setDriveMotors(0.0, 0.0);
      } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S + ARM_EXTEND_TIME_S) {
        setArmMotor(-ARM_OUTPUT_POWER);
        setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
        setDriveMotors(0.0, 0.0);
      } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S + ARM_EXTEND_TIME_S + AUTO_DRIVE_TIME) {
        setArmMotor(0.0);
        setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
        setDriveMotors(AUTO_DRIVE_SPEED, 0.0);
      } else {
        setArmMotor(0.0);
        setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
        setDriveMotors(0.0, 0.0);
      }

    }

    SmartDashboard.putNumber("wheel speeds", m_autoBalance.autoBalanceRoutine());

  }

   /**
   * Used to remember the last game piece picked up to apply some holding power.
   */
  static final int CONE = 1;
  static final int CUBE = 2;
  static final int NOTHING = 3;
  int lastGamePiece;

  @Override
  public void teleopInit() {
    driveLeftSpark.setIdleMode(IdleMode.kCoast);
    driveLeftVictor.setNeutralMode(NeutralMode.Coast);
    driveRightSpark.setIdleMode(IdleMode.kCoast);
    driveRightVictor.setNeutralMode(NeutralMode.Coast);

    lastGamePiece = NOTHING;
  }

  @Override
  public void teleopPeriodic() {
    double armPower;
     armPower = -armJoystick.getRawAxis(1);
     
    if (armPower < -ARM_OUTPUT_POWER ) {
      // lower the arm
      armPower = -ARM_OUTPUT_POWER;
    } else if (armPower > ARM_OUTPUT_POWER) {
      // raise the arm
      armPower = ARM_OUTPUT_POWER;
    } 
    /*else {
      // do nothing and let it sit where it is
      armPower = 0.0;
    }
  */
    setArmMotor(armPower);
  
    double intakePower;
    int intakeAmps;
    if (armJoystick.getRawButton(1)) {
      // cube in or cone out
      intakePower = INTAKE_OUTPUT_POWER;
      intakeAmps = INTAKE_CURRENT_LIMIT_A;
      lastGamePiece = CUBE;
    } else if (armJoystick.getRawButton(2)) {
      // cone in or cube out
      intakePower = -INTAKE_OUTPUT_POWER;
      intakeAmps = INTAKE_CURRENT_LIMIT_A;
      lastGamePiece = CONE;
    } else if (lastGamePiece == CUBE) {
      intakePower = INTAKE_HOLD_POWER;
      intakeAmps = INTAKE_HOLD_CURRENT_LIMIT_A;
    } else if (lastGamePiece == CONE) {
      intakePower = -INTAKE_HOLD_POWER;
      intakeAmps = INTAKE_HOLD_CURRENT_LIMIT_A;
    } else {
      intakePower = 0.0;
      intakeAmps = 0;
    }
    setIntakeMotor(intakePower, intakeAmps);

    /*
     * Negative signs here because the values from the analog sticks are backwards
     * from what we want. Forward returns a negative when we want it positive.
     */
    //setDriveMotors((-driverJoystick.getRawAxis(1))*.2, (-driverJoystick.getRawAxis(4))*.2);
    setDriveMotors((-driverJoystick.getRawAxis(1)), (-driverJoystick.getRawAxis(4)));
  }
}
