package frc.robot.Sensors.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Sensors.Gyro.Gyro;
import frc.robot.Sensors.Vision.VisionManager;

public class BeforeField extends CommandBase {

    VisionManager vision;
            
    public BeforeField() {
        vision = VisionManager.getInstance();
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        Gyro.getInstance().setAngle(vision.getEstimatedGyro().getDegrees()+180);
    }



}
