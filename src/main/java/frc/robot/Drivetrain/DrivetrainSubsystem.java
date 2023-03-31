package frc.robot.Drivetrain;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configuration.ControllableConfiguration;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Sensors.Gyro.Gyro;

public class DrivetrainSubsystem extends SubsystemBase {

    private final SwerveDriveKinematics kinematics;

    private final SwerveModule frontLeft = new SwerveModule(
        "Front Left", 
        this, 
        DriveConstants.frontLeftDriveID, 
        DriveConstants.frontLeftTurnID, 
        DriveConstants.frontLeftAbsID, 
        DriveConstants.frontLeftOffset);

    private final SwerveModule frontRight = new SwerveModule(
        "Front Right", 
        this, 
        DriveConstants.frontRightDriveID, 
        DriveConstants.frontRightTurnID, 
        DriveConstants.frontRightAbsID, 
        DriveConstants.frontRightOffset);

    private final SwerveModule backLeft = new SwerveModule(
        "Back Left", 
        this, 
        DriveConstants.backLeftDriveID, 
        DriveConstants.backLeftTurnID, 
        DriveConstants.backLeftAbsID, 
        DriveConstants.backLeftOffset);
    
    private final SwerveModule backRight = new SwerveModule(
        "Back Right", 
        this, 
        DriveConstants.backRightDriveID, 
        DriveConstants.backRightTurnID, 
        DriveConstants.backRightAbsID, 
        DriveConstants.backRightOffset);

    private SwerveModule[] modules = new SwerveModule[]{frontLeft, frontRight, backLeft, backRight};
    private SwerveModulePosition[] modulePositions = new SwerveModulePosition[modules.length];
    private SwerveModuleState[] moduleStates = new SwerveModuleState[modules.length];

    private final Gyro gyro;

    private final SwerveDriveOdometry odometry;

    private final HashMap<String, ControllableConfiguration> configurations = new HashMap<>();

    public DrivetrainSubsystem() {
        for (int i = 0; i < modules.length; i++) {
            modulePositions[i] = modules[i].getSwerveModulePosition();
            moduleStates[i] = modules[i].getDesiredState();
        }

        kinematics = new SwerveDriveKinematics(new Translation2d[]{
            new Translation2d(ModuleConstants.kModuleToCenter, ModuleConstants.kModuleToCenter),
            new Translation2d(ModuleConstants.kModuleToCenter, -ModuleConstants.kModuleToCenter),
            new Translation2d(-ModuleConstants.kModuleToCenter, ModuleConstants.kModuleToCenter),
            new Translation2d(-ModuleConstants.kModuleToCenter, -ModuleConstants.kModuleToCenter)            
        });

        gyro = Gyro.getInstance();

        odometry = new SwerveDriveOdometry(kinematics, gyro.getAngle(), modulePositions);

        configurations.putAll(Map.ofEntries(
            Map.entry("ClosedLoop", new ControllableConfiguration("Drivetrain", "ClosedLoop", true)),
            Map.entry("Enabled", new ControllableConfiguration("Subsystems", "Drivetrain Enabled", true))
        ));
    }

    @Override
    public void periodic() {
        if (!(Boolean) configurations.get("Enabled").getValue()) {
            stop();
        }

        // TODO: Implement Vision
        odometry.update(gyro.getAngle(), modulePositions);
    }

    public boolean getClosedLoopEnabled() {
        return (Boolean) configurations.get("ClosedLoop").getValue();
    }

    public void drive(ChassisSpeeds speeds) {
        moduleStates = kinematics.toSwerveModuleStates(speeds);
        for (int i = 0; i < modules.length; i++) {
            moduleStates[i] = SwerveModuleState.optimize(moduleStates[i], modules[i].getModuleAngle());
            modules[i].setDesiredState(moduleStates[i]);
        }
    }

    public void stop() {
        for (SwerveModule m : modules) {
            m.stop();
        }
    }

    /**
     * Sets the drivetrain to point in an X position
     * Used to minimize movement
     * 
     * @param b Whether to put the drivetrain in an X or not
     */
    public void frictionBrake() {
        double[] a = new double[] { 45, -45, -45, 45 };
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(a[i])));
        }
    }
}
