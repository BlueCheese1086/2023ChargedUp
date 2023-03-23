package frc.robot.Drivetrain.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Drivetrain.DrivetrainSubsystem;
import frc.robot.Sensors.Field.PositionManager;
import frc.robot.Sensors.Gyro.Gyro;

public class AssistedDrive extends CommandBase {
    

    private final DoubleSupplier x_trans;
    private final DoubleSupplier y_trans;

    private final DrivetrainSubsystem drive;

    private final Gyro gyro = Gyro.getInstance();

    private final PIDController controller = new PIDController(1, 0, 0);

    public AssistedDrive(DrivetrainSubsystem d, DoubleSupplier x, DoubleSupplier y, DoubleSupplier r) {
        x_trans = x;
        y_trans = y;
        drive = d;
        addRequirements(drive);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {

        Pose2d nearestScore = PositionManager.getInstance().poseOfClosestScoring().toPose2d();
        Pose2d currentPos = PositionManager.getInstance().getRobotPose();


        double m1 = (nearestScore.getY()-currentPos.getY())/(nearestScore.getX()-currentPos.getX());
        double m2 = Math.tan(Gyro.getInstance().getAngle().getRadians());

        Rotation2d theta = new Rotation2d(Math.atan((m1-m2)/(1+m1*m2)));

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            y_trans.getAsDouble()*-DriveConstants.MAX_LINEAR_VELOCITY, 
            x_trans.getAsDouble()*DriveConstants.MAX_LINEAR_VELOCITY, 
            // rot.getAsDouble()*DriveConstants.MAX_TURN_VELOCITY,
            controller.calculate(theta.getRadians()),
            gyro.getAngle().plus(new Rotation2d(DriverStation.getAlliance() == Alliance.Red ? 0.0 : Math.PI))
        );
        
        drive.drive(speeds);
        // System.out.println(theta);
        // drive.drive(new ChassisSpeeds(
        //     y_trans.getAsDouble()*DriveConstants.MAX_LINEAR_VELOCITY, 
        //     x_trans.getAsDouble()*DriveConstants.MAX_LINEAR_VELOCITY, 
        //     rot.getAsDouble()*DriveConstants.MAX_TURN_VELOCITY
        // ));
    }

    @Override
    public void end(boolean interr) {
        drive.drive(
            new ChassisSpeeds(0.0, 0.0, 0.0)
        );
    }

}