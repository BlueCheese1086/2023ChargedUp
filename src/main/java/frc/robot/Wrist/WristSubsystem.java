/*package frc.robot.Wrist;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;

public class WristSubsystem extends SubsystemBase {
    
    public CANSparkMax wrist;

    public RelativeEncoder enc;

    public SparkMaxPIDController controller;

    public WristSubsystem(int wristID, boolean inStartingPos) {
        wrist = new CANSparkMax(wristID, MotorType.kBrushless);

        wrist.restoreFactoryDefaults();

        enc = wrist.getEncoder();
        enc.setPositionConversionFactor(360 / WristConstants.GEARBOX_RATIO);
        if (inStartingPos) enc.setPosition(0.0);

        controller = wrist.getPIDController();
        controller.setP(WristConstants.kP);
        controller.setI(WristConstants.kI);
        controller.setD(WristConstants.kD);
        controller.setFF(WristConstants.kFF);
    }

}
*/