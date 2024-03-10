// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  //Motors
  private final PWMVictorSPX frontLeftMotor = new PWMVictorSPX(1);
  private final PWMVictorSPX backLeftMotor = new PWMVictorSPX(2);
  private final PWMVictorSPX frontRightMotor = new PWMVictorSPX(3);
  private final PWMVictorSPX backRightMotor = new PWMVictorSPX(4); 

  private final CANSparkMax pivotMotor = new CANSparkMax(1, MotorType.kBrushless);
  private final DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(0);
  private final PIDController pivotPID = new PIDController(0, 0, 0);

  private final CANSparkMax intakeMotor = new CANSparkMax(2, MotorType.kBrushless);

  private final TalonFX leftshootMotor = new TalonFX(5);
  private final TalonFX rightshootMotor = new TalonFX(4);

  private final CANSparkMax leftclimbMotor = new CANSparkMax(6, MotorType.kBrushless);
  private final CANSparkMax rightclimbMotor = new CANSparkMax(3, MotorType.kBrushless);


  private DifferentialDrive m_drive;

  //Controls
  private final Joystick rightstick = new Joystick(0);
  private final Joystick leftstick = new Joystick(1);
  private final XboxController xbox = new XboxController(2);

  //FUNCTIONS
  //PIVOT
  public void setPivotSpeed(double speed){
    pivotMotor.set(speed);
  }
  public void stopPivot(){
    pivotMotor.set(0);
  }
  public void setPivotToAngle(double setpoint){
    double pivotEncoderAngle = pivotEncoder.getDistance();
    setPivotSpeed(pivotPID.calculate(pivotEncoderAngle, setpoint));
  }
  public void resetPivotEncoder(){
    pivotEncoder.reset();
  }

  //INTAKE
  public void setIntakeSpeed(double speed){
    intakeMotor.set(speed);
  }
  public void stopIntake(){
    intakeMotor.set(0);
  }

  //SHOOTER
  public void setShooterSpeed(double speed){
    leftshootMotor.set(speed);
  }
  public void stopShooter(){
    leftshootMotor.set(0);
  }

  //CLIMBER
  public void setClimberSpeed(double speed){
    leftclimbMotor.set(speed);
    rightclimbMotor.set(speed);  
  }
  public void stopClimber(){
    leftclimbMotor.set(0);
    rightclimbMotor.set(0);
  }

  //AUTO
  // public void shootAndCrossLine(){
  //   new SequentialCommandGroup(
  //     //spin shooter, wait 3s, run intake, wait 3s
  //     new InstantCommand(() -> setShooterSpeed(.25)), new WaitCommand(3), new InstantCommand(() -> setIntakeSpeed(-.5)), new WaitCommand(3),
  //     //stop shooter and intake
  //     new InstantCommand(() -> stopShooter()), new InstantCommand(() -> stopIntake()),
  //     //drive, wait 3s, stop driving
  //     new InstantCommand(() -> m_drive.tankDrive(.5, .5)), new WaitCommand(3), new InstantCommand(() -> m_drive.tankDrive(0, 0))
  //   );
  // }




  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    //DRIVE
    frontRightMotor.setInverted(true);
    //backRightMotor.setInverted(true);
    frontLeftMotor.addFollower(backLeftMotor);
    frontRightMotor.addFollower(backRightMotor);
    m_drive = new DifferentialDrive(frontLeftMotor, frontRightMotor);

    //INTAKE
    intakeMotor.setInverted(false);

    //PIVOT
    pivotMotor.setInverted(false);

    //SHOOTER
    leftshootMotor.setInverted(false);
    rightshootMotor.setControl(new Follower(5, true));

    //CLIMBER
    leftclimbMotor.setInverted(false);
    rightclimbMotor.setInverted(true);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
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
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //DRIVE
    m_drive.tankDrive(leftstick.getY(), rightstick.getY());

    //PIVOT
    if (xbox.getYButton()){
      setPivotToAngle(.5);
    } else {
      setPivotSpeed(xbox.getRightY());
    }

    //INTAKE
    setIntakeSpeed(xbox.getLeftTriggerAxis() - xbox.getRightTriggerAxis());

    //SHOOTER
    if (xbox.getAButton()){
      setShooterSpeed(.25);
    } else if (xbox.getBButton()){
      setShooterSpeed(.5);
    } else {
      stopShooter();
    }

    //CLIMBER
    if (xbox.getBackButton()){
      setClimberSpeed(.5);
    } else if (xbox.getStartButton()){
      setClimberSpeed(-.5);
    } else{
      stopClimber();
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
