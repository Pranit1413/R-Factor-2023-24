// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.SpeedConstants;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */

  private static final WPI_TalonSRX FrontLeft = new WPI_TalonSRX(MotorConstants.leftFront);
  private static final WPI_TalonSRX BackLeft = new WPI_TalonSRX(MotorConstants.leftBack);
  private static final WPI_TalonSRX CenterLeft = new WPI_TalonSRX(MotorConstants.leftCenter);
  private static final WPI_TalonSRX FrontRight = new WPI_TalonSRX(MotorConstants.rightFront);
  private static final WPI_TalonSRX BackRight = new WPI_TalonSRX(MotorConstants.rightBack);
  private static final WPI_TalonSRX CenterRight = new WPI_TalonSRX(MotorConstants.rightCenter);

  public static final MotorControllerGroup LeftGroup = new MotorControllerGroup(FrontLeft, BackLeft, CenterLeft);
  public static final MotorControllerGroup RightGroup = new MotorControllerGroup(FrontRight, BackRight, CenterRight);
  
  public final DifferentialDrive Drive = new DifferentialDrive(LeftGroup, RightGroup);
  

  private double lsp = 0.5;
  private double rsp;
  private double yaw;
  public boolean autoBalanceZMode;
  public boolean autoBalanceYMode;
  public static int currentSetpoint;

  public final static AHRS navX = new AHRS(SPI.Port.kMXP);

  public DriveSubsystem() {
    FrontLeft.configFactoryDefault();
    BackLeft.configFactoryDefault();
    CenterLeft.configFactoryDefault();
    FrontRight.configFactoryDefault();
    BackRight.configFactoryDefault();
    CenterRight.configFactoryDefault();
    navX.reset();
    LeftGroup.setInverted(true);
    Drive.setSafetyEnabled(false);
    currentSetpoint = 4;
    // SmartDashboard.putNumber("Extender Setpoint", currentSetpoint);
    // super(new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD));
    // getController().setTolerance(CameraConstants.AngleTolerance);
    // setSetpoint(SpeedConstants.driveSpeed);
  }

  public void arcadeDrive(double leftSpeed, double rightSpeed){
    this.lsp = leftSpeed;
    this.rsp = rightSpeed;
    Drive.arcadeDrive(lsp, rsp);
  }

  public void Drive1M(double leftSpeed, double rightSpeed){
    Drive.arcadeDrive(leftSpeed, rightSpeed);
  }

  public static void visionturn(double value , double negvalue){
   // Drive.arcadeDrive(0 gyro_Controller.calculate(yaw));
  }

  //public static void driveCartesian(int y, double rotateToAngleRate){
    
  // }

  public void balance(double zAxisRate, double yAxisRate, double twist) {
    
  }


  public void setpoint(PIDController turnController, double targetAngle){
    //Drive.arcadeDrive(0, gyro_Controller.calculate(yaw));
  }

  @Override
  public void periodic(){
    //  this.yaw = getYaw();
    SmartDashboard.putNumber("voltage1", FrontLeft.getBusVoltage());
    SmartDashboard.putNumber("voltage2", CenterLeft.getBusVoltage());
    SmartDashboard.putNumber("voltage3", BackLeft.getBusVoltage());
    SmartDashboard.putNumber("voltage4", FrontRight.getBusVoltage());
    SmartDashboard.putNumber("voltage5", CenterRight.getBusVoltage());
    SmartDashboard.putNumber("voltage6", BackRight.getBusVoltage());
    
    SmartDashboard.putNumber("pitch", navX.getPitch());
  }

  public void SetMax(double Max){
    Drive.setMaxOutput(SpeedConstants.driveSpeed);
  }
  public static boolean setAutoBalanceYMode(boolean autoBalanceYMode){
    return autoBalanceYMode;
  }
  public static boolean setAutoBalanceZMode(boolean autoBalanceZMode){
    return autoBalanceZMode;
  }

  public static void turnDrive(double driveOutput, double turnOutput) {
  }

  public static void stop(double leftSpeed, double rightSpeed){
  }
  public void incrementSetPoint() {
    currentSetpoint = Math.min(currentSetpoint + 1, 4);
  }
  public void decrementSetPoint() {
    currentSetpoint = Math.max(currentSetpoint - 1, 0);
  }
  public int getCurrentSetPoint() {
    return currentSetpoint;
  }
  public void changeSetpoint(int newSetPoint) {
    if (newSetPoint <= 4 && newSetPoint >= 0) {
      currentSetpoint = newSetPoint;
      SmartDashboard.putNumber("setpoint", newSetPoint);
    }
  }
  public void PIDControl(double output, double speed){
    
  }
  // public static double getLeftDistance() {
  //   return leftDriveEncoder.getDistance() / 1000; // Multiply by 1000 to convert from millimeters to meters
  // }
  // public static double getRightDistance() {
  //   return rightDriveEncoder.getDistance() / 1000; // Multiply by 1000 to convert from millimeters to meters
  // }
  

  // public void drive1M(double lsp, double rsp) {
  //   Drive1M.arcadeDrive(lsp, rsp);
  // }
}