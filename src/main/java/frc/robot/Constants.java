package frc.robot;
import edu.wpi.first.math.util.Units;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public final class primaryJoystick {
        public static final int primaryStick = 0;
        public static final int brake = 1;
        public static final int robot0 = 5;
        public static final int robotLeft = 3;
        public static final int robotRight = 4;
        public static final int visionActivate = 7;
        // public static final int  = 9;  
        public static final int boost = 2;
    }
    public final class secondaryJoystick {
        public static final int secondaryStick = 1;
        public static final int extend = 6;
        public static final int retract = 4;
        // public static final int turntable0 = 1;
        public static final int turntableright = 12;
        public static final int turntableleft = 11;
        public static final int armUp = 5;
        // public static final int  = 9;  
        public static final int gripperOpen = 2;
        public static final int gripperClose = 1;
        public static final int armDown = 3;
    }
    // public final class JoystickConstants {
    //     public static final int primaryStick = 0;
    //     public static final int brake = 1;
    //     // public static final int robot0 = 2;
    //     public static final int rotateClockwise = 8;
    //     public static final int rotateAnticlockwise = 10;
    //     public static final int visionActivateS = 0;
    //     public static final int extend = 3;
    //     public static final int retract = 4;
    //     public static final int armUp = 5;
    //     public static final int armDown = 6;
    //     public static final int gripperOpen = 7;
    //     public static final int gripperClose = 9;
    //     public static final int robotLeft = 11;
    //     public static final int robotRight = 12;
    //     public static final int robot0 = 2;

    // }

    public final class MotorConstants{
        public static final int leftFront = 4;
        public static final int leftBack = 5;
        public static final int leftCenter = 3;
        public static final int rightFront = 6;
        public static final int rightBack = 2;
        public static final int rightCenter = 1;
        public static final int turntable1 = 9; 
        public static final int telescopic1 = 8;
        public static final int telescopic2 = 7; 
        public static final int extend1 = 10;
        public static final int gripper1 = 11;
        public static final int gripper2 = 12;
    }

    public final class SpeedConstants{
        public static final double driveSpeed = 0.9;
        public static final double turnSpeed = 0.2;
        public static final double telescopicSpeed = 0.3;
        public static final double extendSpeed = 0.6; 
        public static final double retractSpeed = 0.4;
        public static final double gripperSpeed = 0.35;
        public static final double rotateSpeed = 0.75;
    }

    public final class PIDConstants{
        public static final double kP = 0.007;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double rotationkP = 0.0175;
        public static final double LINEAR_P = 0.1;
        public static final double LINEAR_D = 0.0;
        public static final double ANGULAR_P = 0.1;
        public static final double ANGULAR_D = 0.0;
    }

    public final static class CameraConstants{
        public static final String USBcam1 = "Microsoft_LifeCam_HD-3000";
        public static final double TRACKED_TAG_ROATION_KP = 0.0175;
        public static final double TRACKED_TAG_DISTANCE_DRIVE_KP = 0.3; // P (Proportional) constant of a PID loop
        public static final double TRACKED_TAG_AREA_DRIVE_KP = 0.2; // P (Proportional) constant of a PID loop
        public static final double APRILTAG_POWER_CAP = 0.75;
        public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
        public static final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
        public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);
        public static final double GOAL_RANGE_METERS = Units.feetToMeters(1);
        public static final double DistanceTolerance = 1;
        public static final double AngleTolerance = 5;
        public static final double DRIVE_TURNING_THRESHOLD_DEGREES = 0;
    }

     // DriveSubsystem constants
    //  public static final double kOffBalanceAngleThresholdDegrees = 0;
    //  public static final double kOnBalanceAngleThresholdDegrees = 0;
    //  public static final double kWaitTime = 0.005; // seconds
    //  public static final double kMaxTurnSpeed = 0.6;
    // public static final double TRACKED_TAG_DISTANCE_DRIVE_KP = 0;
    // public static final double APRILTAG_POWER_CAP = 0;

    public final static class AutoConstants{
        public static final double BALANCE_THRESHOLD = 2.0;
        public static final double BALANCE_KP = 0.05;
        public static final double BALANCE_KD = 0.07;
        public static final double MAX_DRIVE_SPEED = 0.5;
    }
}
