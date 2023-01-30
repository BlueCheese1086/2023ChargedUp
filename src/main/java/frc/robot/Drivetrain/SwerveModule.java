package frc.robot.Drivetrain;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {
    
    //private final DutyCycleEncoder cancoder;
    private final AnalogEncoder oh2one;

    private final CANSparkMax turn;
    private final CANSparkMax drive;

    private final RelativeEncoder turnEnc;
    private final RelativeEncoder driveEnc;

    private final SparkMaxPIDController turnPID;
    private final SparkMaxPIDController drivePID;

    //private final SlewRateLimiter turnSlewRateLimiter;

    private final double offset;

    private SwerveModuleState state;

    public SwerveModule(String name, int driveId, int turnId, int channel, double offset) {

        // Deg/s
        //turnSlewRateLimiter = new SlewRateLimiter();

        this.offset = offset;

        this.state = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0));

        //cancoder = new DutyCycleEncoder(new DigitalInput(channel));
        //cancoder.setPositionOffset(this.offset);
        oh2one = new AnalogEncoder(new AnalogInput(channel));
        oh2one.setPositionOffset(offset);

        turn = new CANSparkMax(turnId, MotorType.kBrushless);
        drive = new CANSparkMax(driveId, MotorType.kBrushless);

        turn.restoreFactoryDefaults();
        drive.restoreFactoryDefaults();

        turn.setIdleMode(IdleMode.kBrake);
        drive.setIdleMode(IdleMode.kBrake);

        turn.setInverted(false);
        drive.setInverted(false);

        turnEnc = turn.getEncoder();
        driveEnc = drive.getEncoder();

        driveEnc.setPosition(0.0);
        turnEnc.setPosition(0.0);

        turnPID = turn.getPIDController();
        drivePID = drive.getPIDController();

        turnPID.setP(ModuleConstants.turnP);
        turnPID.setI(ModuleConstants.turnI);
        turnPID.setD(ModuleConstants.turnD);

        drivePID.setP(ModuleConstants.driveP);
        drivePID.setI(ModuleConstants.driveI);
        drivePID.setD(ModuleConstants.driveD);
        drivePID.setFF(ModuleConstants.driveFF);

        turnEnc.setPositionConversionFactor(360 / ModuleConstants.STEER_RATIO);
        driveEnc.setVelocityConversionFactor(ModuleConstants.WHEEL_CIRCUMPHRENCE / ModuleConstants.DRIVE_RATIO / 60);
        driveEnc.setPositionConversionFactor(ModuleConstants.WHEEL_CIRCUMPHRENCE / ModuleConstants.DRIVE_RATIO);
    
        Shuffleboard.getTab("Drivetrain").addNumber(name, () -> turnEnc.getPosition());

        final ShuffleboardLayout layout = Shuffleboard.getTab("Drivetrain").getLayout(name, BuiltInLayouts.kList)
            .withSize(2, 3);

        layout.addNumber("Target Angle", () -> state.angle.getDegrees())
            .withPosition(0, 0)
            .withSize(2, 1)
            .withWidget(BuiltInWidgets.kTextView);
        layout.addNumber("Current Angle", () -> getCurrentAngle().getDegrees())
            .withPosition(0, 1)
            .withSize(2, 1)
            .withWidget(BuiltInWidgets.kTextView);
        layout.addNumber("Velocity", () -> state.speedMetersPerSecond)
            .withPosition(0, 2)
            .withSize(2, 1)
            .withWidget(BuiltInWidgets.kTextView);
        layout.addNumber("0 to 1", () -> oh2one.getAbsolutePosition())
            .withPosition(0, 2)
            .withSize(2, 1)
            .withWidget(BuiltInWidgets.kTextView);
            layout.addNumber("ENC Angle", () -> turnEnc.getPosition())
            .withPosition(0, 1)
            .withSize(2, 1)
            .withWidget(BuiltInWidgets.kTextView);

        Shuffleboard.getTab("Pids").addNumber(name, () -> getCurrentAngle().getDegrees());
    }

    @Override
    public void periodic() {
    }

    public void initEncoder() {
        turnEnc.setPosition(get0to1Angle().getDegrees());
    }

    public double getDistance() {
        return driveEnc.getPosition();
    }

    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromDegrees(get0to1Angle().getDegrees());
    }

    public Rotation2d get0to1Angle() {

        double unsignedAngle = (360 * (oh2one.getAbsolutePosition() - offset) % 360);

        return Rotation2d.fromDegrees(unsignedAngle);

    }

    public Rotation2d getTurnAngle() {

        double unsignedAngle = turnEnc.getPosition() % 360;

        if (unsignedAngle < 0) unsignedAngle += 360;

        return Rotation2d.fromDegrees(unsignedAngle);

    }

    public void setState(SwerveModuleState in) {

        state = in;

        state = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(turnEnc.getPosition()%360));

        drivePID.setReference(state.speedMetersPerSecond, ControlType.kVelocity);

        turnPID.setReference(
            calculateAdjustedAngle(state.angle.getDegrees(), 
            turnEnc.getPosition()), ControlType.kPosition);
    }

    //calculate the angle motor setpoint based on the desired angle and the current angle measurement
    public double calculateAdjustedAngle(double targetAngle, double currentAngle) {

        double modAngle = currentAngle % 360;

        if (modAngle < 0.0) modAngle += 360;
        
        double newTarget = targetAngle + currentAngle - modAngle;

        if (targetAngle - modAngle > 180) newTarget -= 360;
        else if (targetAngle - modAngle < -180) newTarget += 360;

        return newTarget;

    }

    public SwerveModuleState getState() {
        return state;
    }

}
