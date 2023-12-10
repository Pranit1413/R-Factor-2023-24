// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
public class telescopicSubsystem extends SubsystemBase {
  /** Creates a new TelescopicSubsystem. */
  public final CANSparkMax extendarm1 = new CANSparkMax(MotorConstants.extend1, MotorType.kBrushless);
  private RelativeEncoder encoder = extendarm1.getEncoder();

  public telescopicSubsystem() {
    extendarm1.restoreFactoryDefaults();
  }

  public void extendingArm(double speed) {
    extendarm1.set(speed);
  }
  public void retractingArm(double speed) {
    extendarm1.set(speed);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
