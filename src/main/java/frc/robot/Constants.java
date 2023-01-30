package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {

    public static final class DriveConstants {
        public static final int frontLeftTurnID = 3;
        public static final int frontLeftDriveID = 4;
        public static final int frontLeftCancoderID = 6;
        public static final double frontLeftOffset = 0.590;

        public static final int frontRightTurnID = 5;
        public static final int frontRightDriveID = 6;
        public static final int frontRightCancoderID = 0;
        public static final double frontRightOffset = 0.084;

        public static final int backLeftTurnID = 7;
        public static final int backLeftDriveID = 8;
        public static final int backLeftCancoderID = 2;
        public static final double backLeftOffset = 0.504;

        public static final int backRightTurnID = 1;
        public static final int backRightDriveID = 2;
        public static final int backRightCancoderID = 4;
        public static final double backRightOffset = 0.009;

        //m/s
        public static final double MAX_LINEAR_VELOCITY = Units.feetToMeters(14);
        //rad/s
        public static final double MAX_TURN_VELOCITY = 11;
    }
    
    public static class ModuleConstants {

        public static final double STEER_RATIO = 12.8;
        public static final double DRIVE_RATIO = 8.16;

        public static final double WHEEL_CIRCUMPHRENCE = Units.inchesToMeters(4 * Math.PI);

        public static final double kModuleToModuleDistance = Units.inchesToMeters(19.750);
        public static final double kModuleToCenter = kModuleToModuleDistance / 2;

        public static final double driveP = 0.01;
        public static final double driveI = 0.00;
        public static final double driveD = 0.0;
        public static final double driveFF = 1.96;

        public static final double turnP = .01;
        public static final double turnI = 0.0;
        public static final double turnD = 0.005;

    }

    public static final class VisionConstants {
        // Meters
        public static final double cameraHeight = Units.inchesToMeters(4.5);//1;

        // Camera angle offset
        public static final double cameraOffset = 0.0;
    }

    public static final class ElevatorConstants {
        
        public static final int leftID = 0;
        public static final int rightID = 0;

        public static final double GEARBOX_RATIO = 3;

        public static final double SPOOL_RADIUS = Units.inchesToMeters(1.5);

        public static final double MAX_HEIGHT = Units.inchesToMeters(65);

        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kFF = 0.0;

    }

}
