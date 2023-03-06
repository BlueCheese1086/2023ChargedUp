package frc.robot.Arm;

public class ArmState {

    public final double angle;
    public final double velocity;
    public final double abs;

    public ArmState(double angle, double velocity, double abs) {
        this.angle = angle;
        this.velocity = velocity;
        this.abs = abs;
    }
    
}
