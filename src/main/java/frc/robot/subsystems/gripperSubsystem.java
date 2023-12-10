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

public class gripperSubsystem extends SubsystemBase {
  /** Creates a new TelescopicSubsystem. */
  public static final CANSparkMax gripper1 = new CANSparkMax(MotorConstants.gripper1, MotorType.kBrushless);
  public static final CANSparkMax gripper2 = new CANSparkMax(MotorConstants.gripper2, MotorType.kBrushless);

  public final static MotorControllerGroup gripperGroup = new MotorControllerGroup(gripper1, gripper2);

  public gripperSubsystem() {
    gripper1.restoreFactoryDefaults();
    gripper2.restoreFactoryDefaults();
    gripper2.setInverted(true);
  }

  public void movingGripper(double speed) {
    gripperGroup.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
