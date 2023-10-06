package frc.robot.SparkMaxUtils;

import java.util.Objects;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import frc.robot.SparkMaxUtils.Configurations.SparkMaxAbsoluteConfiguration;
import frc.robot.SparkMaxUtils.Configurations.SparkMaxConfiguration;
import frc.robot.SparkMaxUtils.Configurations.SparkMaxPIDConfiguration;
import frc.robot.SparkMaxUtils.Configurations.SparkMaxRelativeConfiguration;
import frc.robot.SparkMaxUtils.Configurations.SparkMaxPIDConfiguration.SensorFeedback;

public class SparkMax extends CANSparkMax {

    private final String name;

    private final SparkMaxConfiguration config;
    private final SparkMaxRelativeConfiguration relConfig;
    private final SparkMaxAbsoluteConfiguration absConfig;
    private final SparkMaxPIDConfiguration pidConfig;

    private IdleMode idle;
    
    public SparkMax(String name, SparkMaxConfiguration config, SparkMaxRelativeConfiguration relConfig, SparkMaxPIDConfiguration pidConfig) {
        super(config.ID, config.MOTOR_TYPE);
        this.name = name;
        this.config = config;
        this.relConfig = relConfig;
        this.absConfig = null;
        this.pidConfig = pidConfig;

        this.configure();

        SparkMaxManager.addMax(this);
    }

    public SparkMax(String name, SparkMaxConfiguration config, SparkMaxRelativeConfiguration relConfig, SparkMaxPIDConfiguration pidConfig, SparkMaxAbsoluteConfiguration absConfig) {
        super(config.ID, config.MOTOR_TYPE);
        this.name = name;
        this.config = config;
        this.relConfig = relConfig;
        this.absConfig = absConfig;
        this.pidConfig = pidConfig;

        this.configure();

        SparkMaxManager.addMax(this);
    }

    public void configure() {
        super.restoreFactoryDefaults();
        super.setIdleMode(config.IDLE_MODE);
        super.setInverted(config.INVERTED);
        super.setSmartCurrentLimit(config.SMART_CURRENT_LIMIT);

        RelativeEncoder a = super.getEncoder();
        a.setInverted(relConfig.INVERTED);
        a.setVelocityConversionFactor(relConfig.VELOCITY_CONVERSION_RATE);
        a.setPositionConversionFactor(relConfig.POSITION_CONVERSION_RATE);

        if (!Objects.isNull(absConfig)) {
            AbsoluteEncoder b = super.getAbsoluteEncoder(Type.kDutyCycle);
            b.setInverted(absConfig.INVERTED);
            b.setPositionConversionFactor(absConfig.POSITION_CONVERSION_RATE);
            b.setVelocityConversionFactor(absConfig.VELOCITY_CONVERSION_RATE);
            b.setZeroOffset(absConfig.OFFSET);
        }
        if (!Objects.isNull(pidConfig)) {
            SparkMaxPIDController pid = super.getPIDController();
            pid.setP(pidConfig.kP);
            pid.setI(pidConfig.kI);
            pid.setD(pidConfig.kD);
            if (pidConfig.feedback == SensorFeedback.absolute) {
                pid.setFeedbackDevice(super.getAbsoluteEncoder(Type.kDutyCycle));
            } else {
                pid.setFeedbackDevice(super.getEncoder());
            }
        }
        idle = config.IDLE_MODE;
        super.burnFlash();
    }

    public boolean isConnected() {
        REVLibError error = super.getLastError();
        return error != REVLibError.kCANDisconnected && 
        error != REVLibError.kInvalidCANId;
    }

    public REVLibError setIdleMode(IdleMode i) {
        if (i.value == idle.value) return REVLibError.kOk;
        idle = i;
        return super.setIdleMode(i);
    }

    public IdleMode getIdleMode() {
        return idle;
    }

    public SparkMaxConfiguration getConfig() {
        return config;
    }

    public String getName() {
        return name;
    }

}