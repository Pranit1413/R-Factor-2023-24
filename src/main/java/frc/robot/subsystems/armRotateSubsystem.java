// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class armRotateSubsystem extends SubsystemBase {
  /** Creates a new TelescopicSubsystem. */
  public static final CANSparkMax rotatearm1 = new CANSparkMax(MotorConstants.telescopic1, MotorType.kBrushless);
  public static final CANSparkMax rotatearm2 = new CANSparkMax(MotorConstants.telescopic2, MotorType.kBrushless);

  private RelativeEncoder encoder = rotatearm2.getEncoder();

  public final static MotorControllerGroup rotateGroup = new MotorControllerGroup(rotatearm1, rotatearm2);

  public armRotateSubsystem() {
    rotatearm1.restoreFactoryDefaults();
    rotatearm2.restoreFactoryDefaults();
  }

  public void Armup(double speed) {
    rotateGroup.set(speed);
  }

  public void Armdown(double speed) {
    rotateGroup.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Encoder", encoder.getPosition());
    SmartDashboard.putNumber("Encoder_Vel", encoder.getVelocity());
  }
}
