package frc.robot.Drivetrain;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Util.DebugPID;

public class SwerveModule extends SubsystemBase {

    private final AnalogEncoder cancoder;

    private final CANSparkMax turn;
    private final CANSparkMax drive;

    private final RelativeEncoder turnEnc;
    private final RelativeEncoder driveEnc;

    private final SparkMaxPIDController turnPID;
    private final SparkMaxPIDController drivePID;

    // private final SlewRateLimiter turnSlewRateLimiter = new SlewRateLimiter(1);
    // private final SlewRateLimiter driveSlewRateLimiter = new SlewRateLimiter(2.0,
    // -Double.MAX_VALUE, 0.0);

    private final double offset;
    private double startTime = System.currentTimeMillis();

    private SwerveModuleState state;

    public SwerveModule(String name, int driveId, int turnId, int channel, double offset) {

        // Deg/s
        // turnSlewRateLimiter = new SlewRateLimiter();

        this.offset = offset;

        this.state = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0));

        cancoder = new AnalogEncoder(channel);// new DutyCycleEncoder(new DigitalInput(channel));
        // cancoder.setPositionOffset(this.offset);

        turn = new CANSparkMax(turnId, MotorType.kBrushless);
        drive = new CANSparkMax(driveId, MotorType.kBrushless);

        turn.restoreFactoryDefaults();
        drive.restoreFactoryDefaults();

        turn.setSmartCurrentLimit(35);
        drive.setSmartCurrentLimit(39);

        // drive.enableVoltageCompensation(12.0);
        // turn.enableVoltageCompensation(12.0);

        turn.setInverted(true);
        drive.setInverted(false);

        turn.setIdleMode(IdleMode.kCoast);
        drive.setIdleMode(IdleMode.kBrake);

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

        turnEnc.setVelocityConversionFactor(360.0 / ModuleConstants.STEER_RATIO / 60);
        turnEnc.setPositionConversionFactor(360.0 / ModuleConstants.STEER_RATIO);
        driveEnc.setVelocityConversionFactor(ModuleConstants.WHEEL_CIRCUMPHRENCE / ModuleConstants.DRIVE_RATIO / 60);
        driveEnc.setPositionConversionFactor(ModuleConstants.WHEEL_CIRCUMPHRENCE / ModuleConstants.DRIVE_RATIO);

        Shuffleboard.getTab("Drivetrain").addNumber(name, () -> turnEnc.getPosition());

        // final ShuffleboardLayout layout = Shuffleboard.getTab("Drivetrain").getLayout(name, BuiltInLayouts.kList)
        //         .withSize(2, 3);

        final ShuffleboardTab layout = Shuffleboard.getTab(name);

        layout.addNumber("Target Angle", () -> state.angle.getDegrees())
                // .withPosition(0, 0)
                // .withSize(2, 1)
                .withWidget(BuiltInWidgets.kGyro).withProperties(Map.ofEntries(
                    Map.entry("Starting angle", 0.0)
                ));
        layout.addNumber("ABS Angle", () -> getCanCoderAngle().getDegrees())
                // .withPosition(0, 1)
                // .withSize(2, 1)
                .withWidget(BuiltInWidgets.kGyro).withProperties(Map.ofEntries(
                    Map.entry("Starting angle", 0.0)
                ));
        layout.addNumber("Velocity", () -> state.speedMetersPerSecond)
                // .withPosition(0, 2)
                // .withSize(2, 1)
                .withWidget(BuiltInWidgets.kTextView);
        layout.addNumber("CanCoder", () -> cancoder.getAbsolutePosition())
                // .withPosition(0, 2)
                // .withSize(2, 1)
                .withWidget(BuiltInWidgets.kTextView);
        layout.addNumber("ENC Angle", () -> getTurnAngle().getDegrees())
                // .withPosition(0, 1)
                // .withSize(2, 1)
                .withWidget(BuiltInWidgets.kGyro).withProperties(Map.ofEntries(
                    Map.entry("Starting angle", 0.0)
                ));

        Shuffleboard.getTab("Pids").addNumber(name, () -> getCanCoderAngle().getDegrees());
        new DebugPID(turnPID, name + "turn");
    }

    @Override
    public void periodic() {
        double elapsed = System.currentTimeMillis() - startTime;
        SmartDashboard.putNumber("Elapsed Time", elapsed);
        if (Robot.isSimulation()) {
            driveEnc.setPosition(driveEnc.getPosition() + elapsed / 1000 * state.speedMetersPerSecond);
        }
        startTime = System.currentTimeMillis();
    }

    /**
     * Gets the current rotation of the module
     * 
     * @return Rotation2d of the current module
     */
    public Rotation2d getCanCoderAngle() {
        double unsignedAngle = (360 * (cancoder.getAbsolutePosition() - offset) % 360);
        if (unsignedAngle < 0)
            unsignedAngle += 360;
        return Rotation2d.fromDegrees(unsignedAngle - 180);
    }

    /**
     * Gets the total distance driven
     * 
     * @return Distance driven (meters)
     */
    public double getDistance() {
        return driveEnc.getPosition();
    }

    /**
     * Gets the motors
     * 
     * @return CANSparkMax[]{drive, turn}
     */
    protected CANSparkMax[] getMotors() {
        return new CANSparkMax[] { drive, turn };
    }

    /**
     * Gets the current module state
     * 
     * @return The current module state
     */
    public SwerveModuleState getState() {
        return state;
    }

    /**
     * Gets the current module angle based on relative encoder
     * 
     * @return Rotation2d of the current module angle
     */
    public Rotation2d getTurnAngle() {
        return Rotation2d.fromDegrees((turnEnc.getPosition() % 360 + 360) % 360);
    }

    public SwerveModuleState getAdjustedState(SwerveModuleState s, Rotation2d moduleAngle) {
        if (Math.abs(s.angle.getDegrees() - moduleAngle.getDegrees()) > 90.0) {
            return new SwerveModuleState(
                -s.speedMetersPerSecond, 
                new Rotation2d(moduleAngle.getRadians() + Math.PI));
        }
        return s;
    }

    /**
     * Initializes the relative encoder
     */
    public void initEncoder() {
        if (Robot.isSimulation())
            return;
        turnEnc.setPosition(getCanCoderAngle().getDegrees() - 180);
    }

    /**
     * Enables raw open loop control of the module
     * 
     * @param speed Drive speed [-1, 1]
     * @param turn  Turn speed [-1, 1]
     */
    public void rawControl(double speed, double turn) {
        this.turn.set(turn);
        this.drive.set(speed);
    }

    /**
     * Resets total distance driven
     * Used for resetting odometry
     */
    public void resetDistance() {
        driveEnc.setPosition(0.0);
    }

    /**
     * Sets the module to the desired state
     * 
     * @param in The desired state
     */
    public void setState(SwerveModuleState in, boolean override) {

        state = in;

        double targetAngle = in.angle.getDegrees();
        double currentAngle = getTurnAngle().getDegrees();
        double delta = ((targetAngle - currentAngle) % 360 + 360) % 360 - 180;

        // if (delta > 90) {
        //     Rotation2d newAngle = getTurnAngle().rotateBy(new Rotation2d(Math.PI));
        //     state = new SwerveModuleState(
        //         -state.speedMetersPerSecond,
        //         newAngle.plus(new Rotation2d())
        //     );
        // }

        state = SwerveModuleState.optimize(in, getTurnAngle());

        // System.out.println(delta);
        SmartDashboard.putNumber("Delta", delta);

        // drive.set(state.speedMetersPerSecond / 4.0);

        if (Robot.isReal()) {
            turnPID.setReference(
                    state.angle.getDegrees(), ControlType.kPosition);

            drivePID.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
            // if (!override) {
            //     System.out.println("Overr");
            //     drivePID.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
            // } else {
            //     drive.set(state.speedMetersPerSecond / 4.0);
            // }
        } else {
            turnEnc.setPosition(in.angle.getDegrees());
        }
    }
}
