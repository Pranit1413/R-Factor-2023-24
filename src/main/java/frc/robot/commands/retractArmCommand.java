// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.telescopicSubsystem;

public class retractArmCommand extends CommandBase {
  /** Creates a new ExtendCommand. */
  public telescopicSubsystem arm;
  public double speed;

  public retractArmCommand(telescopicSubsystem arm, double speed) {
    this.arm = arm;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.retractingArm(-speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

