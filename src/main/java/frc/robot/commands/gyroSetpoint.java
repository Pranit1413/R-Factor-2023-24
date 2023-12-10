// // package frc.robot.commands;

// // import java.util.function.DoubleSupplier;

// // import edu.wpi.first.math.controller.PIDController;
// // import edu.wpi.first.wpilibj2.command.CommandBase;
// // import frc.robot.Constants.CameraConstants;
// // import frc.robot.Constants.PIDConstants;
// // import frc.robot.subsystems.DriveSubsystem;


// // public class gyroSetpoint extends CommandBase {
// //     private final DriveSubsystem driveTrain;
// //     public final PIDController turnController;
// //     private final double targetAngle;
// //     public double getnavXAngle;
// //     private DoubleSupplier lsp;
// //     private DoubleSupplier rsp;
// //     public static double zero;

// //     public gyroSetpoint(DriveSubsystem driveTrain, DoubleSupplier leftspeed, DoubleSupplier rightspeed, PIDController turnController, double targetAngle) {
// //         this.driveTrain = driveTrain;
// //         this.turnController = turnController;
// //         this.targetAngle = targetAngle;
// //         this.lsp = leftspeed;
// //         this.rsp = rightspeed;
// //         addRequirements(driveTrain);
// //     }

// //     @Override
// //     public void initialize() {
// //         turnController.setSetpoint(targetAngle);
// //         turnController.isContinuousInputEnabled();
// //     }

// //     @Override
// //     public void execute() {
// //          driveTrain.arcadeDrive(0,turnController.calculate(DriveSubsystem.navX.getAngle(), targetAngle));
        
// //         zero = DriveSubsystem.navX.getAngle();
// //     //     super(
// //     //     new turnController(PIDConstants.kP, PIDConstants.kI, PIDConstants.kD),
// //     //     driveTrain.getYaw(),
// //     //     // Set reference to target
// //     //     targetAngle,
// //     //     // Pipe output to turn robot
// //     //     output -> driveTrain.arcadeDrive(0, output),
// //     //     // Require the drive
// //     //     driveTrain);

// //     //     turnController().enableContinuousInput(-180, 180);
// //     // // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
// //     // // setpoint before it is considered as having reached the reference
// //     // getController()
// //     //     .setTolerance(CameraConstants.AngleTolerance, CameraConstants.DistanceTolerance);
// //     }

// //     @Override
// //     public void end(boolean interrupted) {
// //         driveTrain.arcadeDrive(0,0);
// //         turnController.disableContinuousInput();
// //     }

// //     @Override
// //     public boolean isFinished() {
// //          return turnController.atSetpoint();
// //         // return getController().atSetpoint();
// //     }
// // }
// package frc.robot.commands;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants;
// import frc.robot.Constants.CameraConstants;
// import frc.robot.Constants.PIDConstants;
// import frc.robot.subsystems.DriveSubsystem;

// public class gyroSetpoint extends CommandBase {

// DriveSubsystem driveTrain; // drivetrain subsystem
//   double degreesToTurn; // the number of degrees we wish to turn
//   double error; // How "incorrect" the current angle of the robot is as its moving
//   double targetAngle; // targetAngle = initial angle + degreesToTurn

//   /** Turns to an angle relative to the current angle using the gyro */
//   public gyroSetpoint(DriveSubsystem driveTrain, double degreesToTurn) {
//     this.driveTrain = driveTrain;
//     // addRequirements(driveTrain);
//     this.degreesToTurn = degreesToTurn;
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     this.targetAngle = degreesToTurn + DriveSubsystem.navX.getAngle();
//     System.out.println("CURRENT ANGLE:" + DriveSubsystem.navX.getAngle());
//     System.out.println("TARGET ANGLE:" + targetAngle);
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     error = targetAngle - DriveSubsystem.navX.getAngle(); // Our target angle, being the angle we want the robot in, vs m_DriveSubsystem.getAngle(), which gets our current angle from the robot

//     double value = error * PIDConstants.kP; // Multiply by scaling factor kp to determine motor percent power between 0 and 100 percent
//     if (Math.abs(value) > 0.75) { // Maximum drive value we want
//       value = Math.copySign(0.75, value);
//     }
//     if (Math.abs(value) < 0.15) { // Minimum drive value we want
//       value = Math.copySign(0.15, value);
//     }
//     // Print statements for debugging //
//     System.out.println("error:" + error);
//     System.out.println("value:" + value);

//     DriveSubsystem.turnDrive(0, value); // drive with the calculated values
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     double leftSpeed = 0;
//     double rightSpeed = 0;
//     DriveSubsystem.stop(leftSpeed, rightSpeed); // Stop the drivetrain motors
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return Math.abs(error) < CameraConstants.AngleTolerance; // End the command when we are within the specified threshold of our target
//   }
// }