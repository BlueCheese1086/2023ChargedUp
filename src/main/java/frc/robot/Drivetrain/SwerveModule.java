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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.SafeSubsystem;
import frc.robot.Constants.ModuleConstants;
import frc.robot.LiveConfiguration.ControllableConfiguration;
import frc.robot.SparkMaxUtils.SparkMax;
import frc.robot.SparkMaxUtils.Configurations.SparkMaxConfiguration;
import frc.robot.SparkMaxUtils.Configurations.SparkMaxPIDConfiguration;
import frc.robot.SparkMaxUtils.Configurations.SparkMaxRelativeConfiguration;
import frc.robot.SparkMaxUtils.Configurations.SparkMaxPIDConfiguration.SensorFeedback;

public class SwerveModule extends SubsystemBase implements SafeSubsystem {

    private final String name;

    private final SparkMax driveMotor;
    private final SparkMax turnMotor;

    private final AnalogEncoder absoluteEncoder;
    private final double absoluteOffset;
    private final RelativeEncoder driveRelEncoder;
    private final RelativeEncoder turnRelEncoder;

    private final SparkMaxPIDController driveController;
    private final SparkMaxPIDController turnController;

    private SwerveModuleState desiredState = new SwerveModuleState();
    private SwerveModuleState actualState = new SwerveModuleState();

    private HashMap<String, ControllableConfiguration> configurations = new HashMap<>();

    private final Timer periodic = new Timer();

    public SwerveModule(String name, DrivetrainSubsystem drivetrain, int driveMotorId, int turnMotorId, int absoluteEncoderId, double absoluteEncoderOffset) {
        this.name = name + " module";
        
        driveMotor = new SparkMax(name + " Drive Motor", 
            new SparkMaxConfiguration(
                driveMotorId, 
                MotorType.kBrushless, 
                false, 
                45, 
                IdleMode.kBrake
            ),
            new SparkMaxRelativeConfiguration(
                false, 
                ModuleConstants.WHEEL_CIRCUMPHRENCE / ModuleConstants.DRIVE_RATIO, 
                ModuleConstants.WHEEL_CIRCUMPHRENCE / ModuleConstants.DRIVE_RATIO / 60.0
            ),
            new SparkMaxPIDConfiguration(
                ModuleConstants.driveP, 
                ModuleConstants.driveI, 
                ModuleConstants.driveD, 
                ModuleConstants.driveFF,
                SensorFeedback.relative
            ));
        turnMotor = new SparkMax(name + " Turn Motor", 
        new SparkMaxConfiguration(
                turnMotorId, 
                MotorType.kBrushless, 
                true, 
                45, 
                IdleMode.kBrake
            ),
            new SparkMaxRelativeConfiguration(
                false, 
                2.0 * Math.PI / ModuleConstants.STEER_RATIO,
                2.0 * Math.PI / ModuleConstants.STEER_RATIO / 60.0
            ),
            new SparkMaxPIDConfiguration(
                ModuleConstants.turnP, 
                ModuleConstants.turnI, 
                ModuleConstants.turnD, 
                1.0,
                SensorFeedback.relative
            ));

        absoluteEncoder = new AnalogEncoder(absoluteEncoderId);
        absoluteOffset = absoluteEncoderOffset;

        driveRelEncoder = driveMotor.getEncoder();
        driveRelEncoder.setPositionConversionFactor(ModuleConstants.WHEEL_CIRCUMPHRENCE / ModuleConstants.DRIVE_RATIO);
        driveRelEncoder.setVelocityConversionFactor(ModuleConstants.WHEEL_CIRCUMPHRENCE / ModuleConstants.DRIVE_RATIO / 60);

        turnRelEncoder = turnMotor.getEncoder();
        turnRelEncoder.setPositionConversionFactor(2.0 * Math.PI / ModuleConstants.STEER_RATIO);
        turnRelEncoder.setVelocityConversionFactor(2.0 * Math.PI / ModuleConstants.STEER_RATIO / 60);

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

        periodic.start();
    }

    @Override
    public void periodic() {
        if (!isSafe()) {
            if (turnMotor.isConnected() && !driveMotor.isConnected()) {
                driveMotor.setIdleMode(IdleMode.kCoast);
            } else {
                driveMotor.setIdleMode(IdleMode.kCoast);
                turnMotor.setIdleMode(IdleMode.kCoast);
                CommandScheduler.getInstance().cancel(this.getCurrentCommand());
            }
        }

        SmartDashboard.putNumber(String.format("Drivetrain/%s/Absolute Encoder", name), absoluteEncoder.getAbsolutePosition());
        SmartDashboard.putNumber(String.format("Drivetrain/%s/Target Angle", name), desiredState.angle.getDegrees());
        SmartDashboard.putNumber(String.format("Drivetrain/%s/Actual Angle", name), actualState.angle.getDegrees());
        SmartDashboard.putNumber(String.format("Drivetrain/%s/Target Velo", name), desiredState.speedMetersPerSecond);
        SmartDashboard.putNumber(String.format("Drivetrain/%s/Actual Velo", name), actualState.speedMetersPerSecond);
        SmartDashboard.putNumber(String.format("Drivetrain/%s/Motor ENC Angle (DEG)", name), Units.radiansToDegrees(turnRelEncoder.getPosition()));

        actualState = new SwerveModuleState(driveRelEncoder.getVelocity(), getModuleAngle());
        
        if (Robot.isSimulation()) {
            driveRelEncoder.setPosition(driveRelEncoder.getPosition() + desiredState.speedMetersPerSecond*periodic.get());
        }

        periodic.reset();
        periodic.start();
    }

    public boolean isSafe() {
        return driveMotor.isConnected() && turnMotor.isConnected();
    }

    public void initializeEncoders() {
        SmartDashboard.putNumber(String.format("Drivetrain/%s/Init Angle", name), Units.radiansToDegrees((absoluteEncoder.getAbsolutePosition() - absoluteOffset) * 2 * Math.PI));
        turnRelEncoder.setPosition((absoluteEncoder.getAbsolutePosition() - absoluteOffset) * 2 * Math.PI - Math.PI);
        driveRelEncoder.setPosition(0.0);
    }

    public Rotation2d getModuleAngle() {
        return new Rotation2d(
            ((turnRelEncoder.getPosition() + Math.PI) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI) - Math.PI
        );
    }

    public double getDistance() {
        return driveRelEncoder.getPosition();
    }

    public double getAdjustedAngle(Rotation2d desiredAngle) {
        return turnRelEncoder.getPosition() + (desiredAngle.minus(getModuleAngle())).getRadians();
    }

    public void setDesiredState(SwerveModuleState s) {
        new InstantCommand(() -> {
            desiredState = s;
            if (Robot.isSimulation()) {
                turnRelEncoder.setPosition(s.angle.getRadians());
                return;
            }
            turnController.setReference(getAdjustedAngle(desiredState.angle), ControlType.kPosition);
            driveController.setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity);
        }, this).schedule();
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