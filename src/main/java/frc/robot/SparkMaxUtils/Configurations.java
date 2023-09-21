package frc.robot.SparkMaxUtils;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Configurations {

    public static class SparkMaxConfiguration {

        public final int ID;
        public final MotorType MOTOR_TYPE;
        public final boolean INVERTED;
        public final int SMART_CURRENT_LIMIT;
        public final IdleMode IDLE_MODE;

        /**
         * 
         * @param id            Motor ID
         * @param type          Motor Type
         * @param motorInverted Motor Inverted
         * @param currentLimit  Current Limit
         * @param idle          Idle Mode
         */
        public SparkMaxConfiguration(
                int id,
                MotorType type,
                boolean motorInverted,
                int currentLimit,
                IdleMode idle) {
            ID = id;
            MOTOR_TYPE = type;
            INVERTED = motorInverted;
            SMART_CURRENT_LIMIT = currentLimit;
            IDLE_MODE = idle;
        }
    }

    public static class SparkMaxRelativeConfiguration {

        public final boolean INVERTED;
        public final double VELOCITY_CONVERSION_RATE;
        public final double POSITION_CONVERSION_RATE;

        /**
         * @param invertedRotation Does the encoder rotate the opposite of the motor
         * @param velocityConv Velocity conversion
         * @param posInv Position conversion
         */
        public SparkMaxRelativeConfiguration(
            boolean invertedRotation,
            double velocityConv,
            double posInv) {
                INVERTED = invertedRotation;
                VELOCITY_CONVERSION_RATE = velocityConv;
                POSITION_CONVERSION_RATE = posInv;
        }

        public static SparkMaxRelativeConfiguration getDefault() {
            return new SparkMaxRelativeConfiguration(false, 1, 1);
        }
    }

    public static class SparkMaxAbsoluteConfiguration {

        public final boolean INVERTED;
        public final double OFFSET;
        public final double VELOCITY_CONVERSION_RATE;
        public final double POSITION_CONVERSION_RATE;

        /**
         * @param invertedRotation Does the encoder rotate the opposite of the motor
         * @param rotationOffset Rotation offset
         * @param velocityConv Velocity conversion
         * @param posInv Position conversion
         */
        public SparkMaxAbsoluteConfiguration(
            boolean invertedRotation,
            double rotationOffset,
            double velocityConv,
            double posInv) {
                INVERTED = invertedRotation;
                OFFSET = rotationOffset;
                VELOCITY_CONVERSION_RATE = velocityConv;
                POSITION_CONVERSION_RATE = posInv;
        }

        public static SparkMaxAbsoluteConfiguration getDefault() {
            return new SparkMaxAbsoluteConfiguration(false, 0, 1, 1);
        }
    }

    public static class SparkMaxPIDConfiguration {

        public enum SensorFeedback {
            relative,
            absolute
        }

        public final double kP;
        public final double kI;
        public final double kD;
        public final double kFF;
        public final SensorFeedback feedback;

        /**
         * @param P Proportional
         * @param I Integral
         * @param D Derivative
         */
        public SparkMaxPIDConfiguration(
            double P,
            double I,
            double D,
            double FF,
            SensorFeedback feedbackDevice) {
                kP = P;
                kI = I;
                kD = D;
                kFF = FF;
                feedback = feedbackDevice;
        }

        public static SparkMaxPIDConfiguration getDefault() {
            return new SparkMaxPIDConfiguration(0, 0, 0, 0, SensorFeedback.relative);
        }
    }
}