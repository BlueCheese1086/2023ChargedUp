package frc.robot.Drivetrain;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class DrivetrainSubsystem extends SubsystemBase {

    /* GYRO */
    private final PigeonIMU pigeon = new PigeonIMU(2);

    /* MODULES */
    private final SwerveModule frontLeft = new SwerveModule(
        "Front Left",
        DriveConstants.frontLeftDriveID,
        DriveConstants.frontLeftTurnID,
        DriveConstants.frontLeftCancoderID,
        DriveConstants.frontLeftOffset
    );
    private final SwerveModule frontRight = new SwerveModule(
        "Front Right",
        DriveConstants.frontRightDriveID,
        DriveConstants.frontRightTurnID,
        DriveConstants.frontRightCancoderID,
        DriveConstants.frontRightOffset
    );;
    private final SwerveModule backLeft = new SwerveModule(
        "Back Left",
        DriveConstants.backLeftDriveID,
        DriveConstants.backLeftTurnID,
        DriveConstants.backLeftCancoderID,
        DriveConstants.backLeftOffset
    );;
    private final SwerveModule backRight = new SwerveModule(
        "Back Right",
        DriveConstants.backRightDriveID,
        DriveConstants.backRightTurnID,
        DriveConstants.backRightCancoderID,
        DriveConstants.backRightOffset
    );;

    /* MODULE MANAGEMENT */
    private final SwerveModule[] modules;
    private SwerveModuleState[] states = new SwerveModuleState[4];

    private final SwerveModulePosition[] modulePositions;

    private final SwerveDriveKinematics kinematics;
    private ChassisSpeeds speeds;

    /* ODOMETRY */
    private final SwerveDriveOdometry odometry;
    private final Field2d field = new Field2d();


    public DrivetrainSubsystem() {        
        modules = new SwerveModule[]{frontLeft, frontRight, backLeft, backRight};
        speeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        
        for (SwerveModule s : modules) {
            s.initEncoder();
        }

        modulePositions = new SwerveModulePosition[]{
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        };

        kinematics = new SwerveDriveKinematics(
            new Translation2d(ModuleConstants.kModuleToCenter, ModuleConstants.kModuleToCenter),
            new Translation2d(ModuleConstants.kModuleToCenter, -ModuleConstants.kModuleToCenter),
            new Translation2d(-ModuleConstants.kModuleToCenter, ModuleConstants.kModuleToCenter),
            new Translation2d(-ModuleConstants.kModuleToCenter, -ModuleConstants.kModuleToCenter));
 
        /* GYRO INIT */
        pigeon.configFactoryDefault();
        pigeon.setYaw(180);

        /* ODOMETRY */
        odometry = new SwerveDriveOdometry(kinematics, getRobotAngle(), modulePositions);

        /* TELEMETRY */
        Shuffleboard.getTab("Field").add(field);
        Shuffleboard.getTab("Field").addNumber("Gyro", ()->getRobotAngle().getDegrees());
    }

    /*
     * Runs every tick
     */
    @Override
    public void periodic() {
        //SmartDashboard.putNumber("Gyro", pigeon.getYaw());

        for (int i1 = 0; i1 < modules.length; i1++) {
            modulePositions[i1] = new SwerveModulePosition(modules[i1].getDistance(), modules[i1].getTurnAngle());
        }
        odometry.update(getRobotAngle(), modulePositions);
    }

    /* SETTERS */

    public void drive(ChassisSpeeds sp) {
        this.speeds = sp;
        states = kinematics.toSwerveModuleStates(speeds, new Translation2d(0.0, 0.0));
        for (int i = 0; i < 4; i++) {
            modules[i].setState(states[i]);
            //modules[i].setState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(0)));
        }
    }

    public void set(SwerveModuleState s, int i) {
        modules[i].setState(s);
    }

    public void stop() {
        this.drive(
            new ChassisSpeeds(0.0, 0.0, 0.0)
        );
    }

    public void setStates(SwerveModuleState[] s) {
        for (int i = 0; i < 4; i++) {
            states[i] = s[i];
            modules[i].setState(states[i]);
        }
    }

    public void resetGyro() {
        pigeon.setYaw(180);
    }

    /* GETTERS */

    public SwerveModuleState[] getStates() {
        return new SwerveModuleState[]{frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState()};
    }

    public Rotation2d getRobotAngle() {
        return Rotation2d.fromDegrees(pigeon.getYaw()%360 - 180);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public Command getFollowCommand() {
        PathPlannerTrajectory traj = PathPlanner.loadPath("1 Piece", new PathConstraints(1, 1));
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                odometry.resetPosition(traj.getInitialPose().getRotation(), modulePositions, traj.getInitialPose());
            }),
            new PPSwerveControllerCommand(
                traj,
                this::getPose,
                this.kinematics, 
                new PIDController(3, 0, 0), 
                new PIDController(3, 0, 0), 
                new PIDController(3, 0, 0), 
                this::setStates, 
                this)
            );
    }

}