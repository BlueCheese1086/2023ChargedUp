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

        public static final double driveP = 0.01;
        public static final double driveI = 0.00;
        public static final double driveD = 0.0;
        public static final double driveFF = 1.96;

        public static final double turnP = 0.05;
        public static final double turnI = 0.0;
        public static final double turnD = 0.005;

    }

    public static final class VisionConstants {
        // Meters
        public static final double cameraHeight = Units.inchesToMeters(4.5);//1;

        // Camera angle offset
        public static final double cameraOffset = 0.0;

        // POSE Estimations
        public static final int POSE_ESTIMATIONS = 30;

        // MAX POSE time kept millis
        public static final double TIME_KEPT = 1500;
    }

    public static final class ElevatorConstants {
        
        public static final int leftID = 12;
        public static final int rightID = 11;
        
        public static final int bottomSwitchID = 0;

        public static final double GEARBOX_RATIO = 11.25;

        public static final double SPOOL_DIAMETER = Units.inchesToMeters(2.5);

        public static final double MAX_HEIGHT = Units.inchesToMeters(65);

        public static final double TOWER_ANGLE_OFFSET = Units.degreesToRadians(90-72);

        public static final double MINIMUM_STARTING_HEIGHT = (int)(Units.inchesToMeters(25)/(SPOOL_DIAMETER*Math.PI))*(double)SPOOL_DIAMETER*Math.PI;

        public static final double TOLERANCE = Units.inchesToMeters(2);

        public static final double kP = 3.0;
        public static final double kI = 0.0005;
        public static final double kD = 0.001;
        public static final double kFF = 0.0;

    }

    public static final class ArmConstants {

        public static final int armId = 23;

        public static final double GEARBOX_RATIO = 140.952380952;

        /**
         * Meters
         */
        public static final double ARM_LENGTH = Units.inchesToMeters(24);

        // Arm Starting angle
        public static final double STARTING_ANGLE = 0.0;
        // Arm Range
        public static final double RANGE = Units.degreesToRadians(170);

        public static final double UPPER_RANGE = RANGE/2.0 - ElevatorConstants.TOWER_ANGLE_OFFSET;
        public static final double LOWER_RANGE = -Units.degreesToRadians(60);

        public static final double TOLERANCE = Units.degreesToRadians(15);

        public static final double kP = 1.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kFF = 0.0;

        public static final double ENC_OFFSET = 4.35 - Math.PI;//2.14 + Math.PI;//2.4;

    }

    public static final class IntakeConstants {
        
        public static final int ID = 31;

        // INTAKE IS REALLY 14 INCHES
        // THIS IS TO MAKE MY LIFE EASIER WITH INVERSE KIN
        public static final double INTAKE_LENGTH = Units.inchesToMeters(18);
    }

    public static class GyroConstants {
        public static final int GYRO_ID = 2;

    }

    public static class WristConstants {

        public static final int LEFT_ID = 22;
        public static final int RIGHT_ID = 21;

        public static final double kP = 1.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kFF = 0.0;

        public static final double UPPER_RANGE = Units.degreesToRadians(90);
        public static final double LOWER_RANGE = Units.degreesToRadians(-45);

        public static final double TOLERANCE = Units.degreesToRadians(10);

        public static final double GEARBOX_RATIO = 0;

        public static final double ENC_OFFSET = 3.40 - Math.PI;

    }

    public static class LEDConstants {

        public static final int PORT = 0;

        public static final int COUNT = 255;

    }

    public static class FieldConstants {

        public static final double SCORING_DISTANCE_Y = Units.inchesToMeters(22);
        public static final double SCORING_DISTANCE_X = Units.inchesToMeters(17);
        public static final double SCORING_DISTANCE_Z = Units.inchesToMeters(12);
        
    }

}
