package frc.robot.Drivetrain;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;
import frc.robot.SparkMaxUtils.SparkMax;

public class SwerveModule extends SubsystemBase {

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

    public SwerveModule(String name, int driveMotorId, int turnMotorId, int absoluteEncoderId, double absoluteEncoderOffset) {
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
    }

    @Override
    public void periodic() {

    }

    public void setState() {
        
    }

}