package frc.robot.Drivetrain.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Configuration.ControllableConfiguration;
import frc.robot.Drivetrain.DrivetrainSubsystem;
import frc.robot.Sensors.Feedback.VisualFeedback;
import frc.robot.Sensors.Feedback.VisualFeedback.LEDMode;
import frc.robot.Sensors.Vision.Vision;
import frc.robot.StateManager.StateManager;
import frc.robot.StateManager.StateManager.Piece;

public class AlignmentDrive extends CommandBase {

    private final DrivetrainSubsystem drivetrain;
    private final StateManager state;
    private final Vision vision;

    private final DoubleSupplier xSupplier;

    private final PIDController controller;

    private final ControllableConfiguration coneOffset;

    public AlignmentDrive(DrivetrainSubsystem d, DoubleSupplier x) {
        drivetrain = d;
        state = StateManager.getInstance();
        vision = Vision.getInstance();
        xSupplier = x;
        controller = new PIDController(0.1, 0, 0.0);
        controller.setTolerance(15.0);
        coneOffset = new ControllableConfiguration("Vision", "ConeOffset", 15.0);
        addRequirements(drivetrain, VisualFeedback.getInstance());
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        ChassisSpeeds speeds;
        if (state.getPieceMode() == Piece.Cube) {
            speeds = new ChassisSpeeds(
                xSupplier.getAsDouble(), 
                0.0,//controller.calculate(vision.getAverage(), 0), 
                controller.calculate(vision.getAverage(), 0)
            );
        } else if (state.getPieceMode() == Piece.Cone && Double.isNaN(vision.getLeftCamTargetRotation()) && !Double.isNaN(vision.getRightCamTargetRotation())) {
            speeds = new ChassisSpeeds(
                xSupplier.getAsDouble(), 
                0.0,//controller.calculate(vision.getAverage(), (Double) coneOffset.getValue()),
                controller.calculate(vision.getAverage(), (Double) coneOffset.getValue())
            );
        } else if (state.getPieceMode() == Piece.Cone && !Double.isNaN(vision.getLeftCamTargetRotation()) && Double.isNaN(vision.getRightCamTargetRotation())) {
            speeds = new ChassisSpeeds(
                xSupplier.getAsDouble(), 
                0.0,//controller.calculate(vision.getAverage(), -(Double) coneOffset.getValue()),
                controller.calculate(vision.getAverage(), -(Double) coneOffset.getValue())
            );
        } else {
            speeds = new ChassisSpeeds(xSupplier.getAsDouble(), 0, 0);
        }
        if (controller.atSetpoint()) {
            VisualFeedback.getInstance().setMode(LEDMode.Good);
        } else {
            VisualFeedback.getInstance().setMode(LEDMode.Bad);
        }
        drivetrain.drive(speeds);
    }
    
}
