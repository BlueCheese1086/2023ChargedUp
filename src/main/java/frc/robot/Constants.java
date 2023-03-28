package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
    
    public static final class DriveConstants {
        public static final int frontLeftTurnID = 3;
        public static final int frontLeftDriveID = 4;
        public static final int frontLeftCancoderID = 3;
        public static final double frontLeftOffset = 0.059;

        public static final int frontRightTurnID = 6;
        public static final int frontRightDriveID = 5;
        public static final int frontRightCancoderID = 2;
        public static final double frontRightOffset = 0.404;

        public static final int backLeftTurnID = 8;
        public static final int backLeftDriveID = 7;
        public static final int backLeftCancoderID = 1;
        public static final double backLeftOffset = 0.514;

        public static final int backRightTurnID = 1;
        public static final int backRightDriveID = 2;
        public static final int backRightCancoderID = 0;
        public static final double backRightOffset = 0.532;

        //m/s
        public static final double MAX_LINEAR_VELOCITY = Units.feetToMeters(16);
        //rad/s
        public static final double MAX_TURN_VELOCITY = 11;
    }
    
    public static class ModuleConstants {

        public static final double STEER_RATIO = 150.0/7.0;
        public static final double DRIVE_RATIO = 6.12;

        public static final double WHEEL_CIRCUMPHRENCE = Units.inchesToMeters(4 * Math.PI);

        public static final double kModuleToModuleDistance = Units.inchesToMeters(19.750);
        public static final double kModuleToCenter = kModuleToModuleDistance / 2;

        public static final double driveP = 0.1;
        public static final double driveI = 0.00;
        public static final double driveD = 0.0;
        public static final double driveFF = 2.96;

        public static final double turnP = 0.05;
        public static final double turnI = 0.0;
        public static final double turnD = 0.005;

        public static final int driveCurrentLimit = 35;
        public static final int turnCurrentLimit = 39;

    }

}