package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
    
    public static final class DriveConstants {
        public static final int frontLeftTurnID = 3;
        public static final int frontLeftDriveID = 4;
        public static final int frontLeftAbsID = 0;
        public static final double frontLeftOffset = 0.059;

        public static final int frontRightTurnID = 6;
        public static final int frontRightDriveID = 5;
        public static final int frontRightAbsID = 1;
        public static final double frontRightOffset = 0.407;

        public static final int backLeftTurnID = 8;
        public static final int backLeftDriveID = 7;
        public static final int backLeftAbsID = 2;
        public static final double backLeftOffset = 0.515;

        public static final int backRightTurnID = 1;
        public static final int backRightDriveID = 2;
        public static final int backRightAbsID = 3;
        public static final double backRightOffset = 0.530;

        //m/s
        public static final double MAX_LINEAR_VELOCITY = Units.feetToMeters(16);
        //rad/s
        public static final double MAX_TURN_VELOCITY = MAX_LINEAR_VELOCITY/ModuleConstants.kModuleHypToCenter;
    }

    public static class ModuleConstants {

        public static final double STEER_RATIO = 150.0/7.0;
        public static final double DRIVE_RATIO = 6.12;

        public static final double WHEEL_CIRCUMPHRENCE = Units.inchesToMeters(3.75 * Math.PI);

        public static final double kModuleToModuleDistance = Units.inchesToMeters(19.750);
        public static final double kModuleToCenter = kModuleToModuleDistance / 2;
        public static final double kModuleHypToCenter = Math.sqrt(kModuleToCenter*kModuleToCenter*2);

        public static final double driveP = 0.05;
        public static final double driveI = 0.00;
        public static final double driveD = 0.0;
        public static final double driveFF = 0.26;

        public static final double turnP = 1;
        public static final double turnI = 0.0;
        public static final double turnD = 0.0;

        public static final int driveCurrentLimit = 35;
        public static final int turnCurrentLimit = 39;

    }

    public static class GyroConstants {
        public static final int GYRO_ID = 2;
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

        public static final double UPPER_RANGE = 1.5;
        public static final double LOWER_RANGE = -1.69;

        public static final double TOLERANCE = Units.degreesToRadians(15);

        public static final double kP = 0.75;
        public static final double kI = 0.0001;
        public static final double kD = 1.0;
        public static final double kFF = 0.0;

        public static final double ENC_OFFSET = 4.18 - Math.PI;

    }

    public static final class IntakeConstants {
        public static final int ID = 31;
        // INTAKE IS REALLY 14 INCHES
        // THIS IS TO MAKE MY LIFE EASIER WITH INVERSE KIN
        public static final double INTAKE_LENGTH = Units.inchesToMeters(18);
    }

    public static class WristConstants {

        public static final int LEFT_ID = 22;
        public static final int RIGHT_ID = 21;

        public static final double kP = 1.0;
        public static final double kI = 0.0;
        public static final double kD = 0.5;
        public static final double kFF = 0.0;

        public static final double UPPER_RANGE = 2.9;
        public static final double LOWER_RANGE = -.47;

        public static final double TOLERANCE = Units.degreesToRadians(10);

        public static final double GEARBOX_RATIO = 1/45;

        public static final double ENC_OFFSET = 3.04967 + Math.PI;

    }

    public static class LEDConstants {

        public static final int PORT = 0;

        public static final int COUNT = 300;

    }

    public static class FieldConstants {

        public static final double SCORING_DISTANCE_Y = Units.inchesToMeters(22);
        public static final double SCORING_DISTANCE_X = Units.inchesToMeters(17);
        public static final double SCORING_DISTANCE_Z = Units.inchesToMeters(12);
        
    }

}