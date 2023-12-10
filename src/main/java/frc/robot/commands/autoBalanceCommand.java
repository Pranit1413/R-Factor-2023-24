// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.SpeedConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.*;

// public class autoBalanceCommand extends CommandBase {
//   private static final double kOffBalanceAngleThresholdDegrees = 10;
//   private static final double kOnBalanceAngleThresholdDegrees = 5;
//   double leftSpeed;
//   double rightSpeed;
//   private final DriveSubsystem driveTrain;

//   public autoBalanceCommand(DriveSubsystem driveTrain, boolean setAutoBalanceYMode, boolean setAutoBalanceZMode) {
//     this.driveTrain = driveTrain;
//     addRequirements(driveTrain);
//   }

//   @Override
//   public void initialize() {
//     DriveSubsystem.setAutoBalanceZMode(false);
//     DriveSubsystem.setAutoBalanceYMode(false);
//   }

//   @Override
//   public void execute() {
//     double zAxisRate = RobotContainer.driveStick.getZ();
//     double yAxisRate = RobotContainer.driveStick.getY();
//     double pitchAngleDegrees = DriveSubsystem.navX.getPitch();
//     double yawAngleDegrees = DriveSubsystem.navX.getYaw();

//     if ( DriveSubsystem.setAutoBalanceZMode(true) && 
//                  (Math.abs(pitchAngleDegrees) >= 
//                   Math.abs(kOffBalanceAngleThresholdDegrees))) {
//                 DriveSubsystem.setAutoBalanceZMode(true);
//             }
//             else if (DriveSubsystem.setAutoBalanceZMode(false) && 
//                       (Math.abs(pitchAngleDegrees) <= 
//                        Math.abs(kOnBalanceAngleThresholdDegrees))) {
//                         DriveSubsystem.setAutoBalanceZMode(false);
//             }
//             if ( DriveSubsystem.setAutoBalanceYMode(true) && 
//             (Math.abs(pitchAngleDegrees) >= 
//              Math.abs(kOffBalanceAngleThresholdDegrees))) {
//            DriveSubsystem.setAutoBalanceYMode(true);
//        }
//        else if (DriveSubsystem.setAutoBalanceYMode(false) && 
//                  (Math.abs(pitchAngleDegrees) <= 
//                   Math.abs(kOnBalanceAngleThresholdDegrees))) {
//                    DriveSubsystem.setAutoBalanceYMode(false);
//        }

   
    
//   }

//   @Override
//   public void end(boolean interrupted) {
    
//   }

//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class autoBalanceCommand extends PIDCommand {
  /** Creates a new LimelightAim. */
  private final DriveSubsystem driveTrain;
  public autoBalanceCommand(DriveSubsystem navX, DriveSubsystem driveTrain) {
    super(
        // The controller that the command will use
        new PIDController(.1, 0, .01),
        // This should return the measurement
        () -> DriveSubsystem.navX.getPitch(),
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          driveTrain.PIDControl(output / 16, 0.3);
        });
    this.driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);

    // Configure additional PID options by calling `getController` here.

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}