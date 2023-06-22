package frc.robot.Drivetrain;

import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Sensors.Field.PositionManager;
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

    private final Gyro gyro;

    private final SwerveDriveOdometry odometry;

    private SwerveAutoBuilder autoBuilder;

    public DrivetrainSubsystem() {
        for (int i = 0; i < modules.length; i++) {
            modulePositions[i] = modules[i].getSwerveModulePosition();
        }

        kinematics = new SwerveDriveKinematics(new Translation2d[]{
            new Translation2d(ModuleConstants.kModuleToCenter, ModuleConstants.kModuleToCenter),
            new Translation2d(ModuleConstants.kModuleToCenter, -ModuleConstants.kModuleToCenter),
            new Translation2d(-ModuleConstants.kModuleToCenter, ModuleConstants.kModuleToCenter),
            new Translation2d(-ModuleConstants.kModuleToCenter, -ModuleConstants.kModuleToCenter)            
        });

        gyro = Gyro.getInstance();

        odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(), modulePositions);

        for (SwerveModule s : modules) {
            s.initializeEncoders();
        }
    }

    public void setEvents(Map<String, Command> entries) {
        Map<String, Command> eventMap = entries;

        autoBuilder = new SwerveAutoBuilder(
            () -> odometry.getPoseMeters(),//PositionManager.getInstance().getRobotPose(), 
            (Pose2d p) -> {
                odometry.resetPosition(gyro.getAngle(), modulePositions, p);
            },
            kinematics,
            new PIDConstants(1.0, 0.0, 0.0),
            new PIDConstants(1.0, 0.0, 0.0), 
            (SwerveModuleState[] s) -> {setStates(s);}, 
            eventMap, 
            true, 
            this);
    }

    @Override
    public void periodic() {

        for (int i = 0; i < modules.length; i++) {
            modulePositions[i] = new SwerveModulePosition(modules[i].getDistance(), modules[i].getModuleAngle());
        }

        odometry.update(gyro.getAngle(), modulePositions);
        PositionManager.getInstance().setRobotPose(odometry.getPoseMeters());
    }

    public void drive(ChassisSpeeds speeds) {
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
        for (int i = 0; i < modules.length; i++) {
            moduleStates[i] = SwerveModuleState.optimize(moduleStates[i], modules[i].getModuleAngle());
            modules[i].setDesiredState(moduleStates[i]);
        }
    }

    public void setStates(SwerveModuleState[] toSet) {
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(toSet[i]);
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

    public Command getPathCommand(String name, PathConstraints p) {
        List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup(name, p.maxVelocity, p.maxAcceleration);
        return new SequentialCommandGroup(
            new FunctionalCommand(
                () -> {},
                () -> {
                    Gyro.getInstance().getGyro().setYaw(path.get(0).getInitialHolonomicPose().getRotation().getDegrees()+180);
                },
                (Boolean f) -> {},
                () -> {
                    return Math.abs(Gyro.getInstance().getAngle().getDegrees() - path.get(0).getInitialHolonomicPose().getRotation().getDegrees()) < 0.3;
                }
            ),
            autoBuilder.fullAuto(path)
        );
    }
}
