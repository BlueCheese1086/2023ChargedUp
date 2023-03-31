package frc.robot.Drivetrain;

import java.util.HashMap;
import java.util.Map;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configuration.ControllableConfiguration;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.SparkMaxUtils.SparkMax;

public class SwerveModule extends SubsystemBase {

    private final DrivetrainSubsystem drivetrain;

    private final SparkMax driveMotor;
    private final SparkMax turnMotor;

    private final AnalogEncoder absoluteEncoder;
    private final double absoluteOffset;
    private final RelativeEncoder driveRelEncoder;
    private final RelativeEncoder turnRelEncoder;

    private final SparkMaxPIDController driveController;
    private final SparkMaxPIDController turnController;

    private SwerveModuleState desiredState;
    private SwerveModuleState actualState;

    private HashMap<String, ControllableConfiguration> configurations = new HashMap<>();

    public SwerveModule(String name, DrivetrainSubsystem drivetrain, int driveMotorId, int turnMotorId, int absoluteEncoderId, double absoluteEncoderOffset) {
        this.drivetrain = drivetrain;
        
        driveMotor = new SparkMax(name + " Drive Motor", driveMotorId, MotorType.kBrushless);
        turnMotor = new SparkMax(name + " Turn Motor", turnMotorId, MotorType.kBrushless);

        driveMotor.restoreFactoryDefaults();
        turnMotor.restoreFactoryDefaults();

        driveMotor.setSmartCurrentLimit(ModuleConstants.driveCurrentLimit);
        turnMotor.setSmartCurrentLimit(ModuleConstants.turnCurrentLimit);

        driveMotor.setIdleMode(IdleMode.kCoast);
        turnMotor.setIdleMode(IdleMode.kCoast);

        absoluteEncoder = new AnalogEncoder(absoluteEncoderId);
        absoluteOffset = absoluteEncoderOffset;

        driveRelEncoder = driveMotor.getEncoder();
        driveRelEncoder.setPositionConversionFactor(ModuleConstants.WHEEL_CIRCUMPHRENCE / ModuleConstants.DRIVE_RATIO);
        driveRelEncoder.setVelocityConversionFactor(ModuleConstants.WHEEL_CIRCUMPHRENCE / ModuleConstants.DRIVE_RATIO / 60);

        turnRelEncoder = turnMotor.getEncoder();
        turnRelEncoder.setPositionConversionFactor(2 * Math.PI / ModuleConstants.STEER_RATIO);
        turnRelEncoder.setVelocityConversionFactor(2 * Math.PI / ModuleConstants.STEER_RATIO / 60);

        driveController = driveMotor.getPIDController();
        driveController.setP(ModuleConstants.driveP);
        driveController.setI(ModuleConstants.driveI);
        driveController.setD(ModuleConstants.driveD);
        driveController.setFF(ModuleConstants.driveFF);

        turnController = turnMotor.getPIDController();
        turnController.setP(ModuleConstants.turnP);
        turnController.setI(ModuleConstants.turnI);
        turnController.setD(ModuleConstants.turnD);

        driveMotor.burnFlash();
        turnMotor.burnFlash();

        configurations.putAll(Map.ofEntries(
            Map.entry("Enabled", new ControllableConfiguration("Drivetrain", name, true))
        ));
    }

    @Override
    public void periodic() {
        turnMotor.setDisabled(!(Boolean)configurations.get("Enabled").getValue());
        driveMotor.setDisabled(!(Boolean)configurations.get("Enabled").getValue());

        actualState = new SwerveModuleState(driveRelEncoder.getVelocity(), new Rotation2d());
    }

    public void initializeEncoders() {
        turnRelEncoder.setPosition((absoluteEncoder.getAbsolutePosition() - absoluteOffset) * 2 * Math.PI - Math.PI);
        driveRelEncoder.setPosition(0.0);
    }

    public Rotation2d getModuleAngle() {
        return new Rotation2d(
            (turnRelEncoder.getPosition() % 2 * Math.PI + 2 * Math.PI) % 2 * Math.PI - Math.PI
        );
    }

    public void setDesiredState(SwerveModuleState s) {
        desiredState = s;
        turnController.setReference(desiredState.angle.getRadians(), ControlType.kPosition);
        if (drivetrain.getClosedLoopEnabled()) {
            driveController.setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity);
        } else {
            driveMotor.set(desiredState.speedMetersPerSecond/DriveConstants.MAX_LINEAR_VELOCITY);
        }
    }

    public void stop() {
        driveMotor.stopMotor();
        turnMotor.stopMotor();
    }

    public SwerveModuleState getDesiredState() {
        return desiredState;
    }

    public SwerveModuleState getActualState() {
        return actualState;
    }

    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(driveRelEncoder.getPosition(), getModuleAngle());
    }

}