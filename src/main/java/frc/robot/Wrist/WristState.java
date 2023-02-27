package frc.robot.Wrist;

/** Add your docs here. */
public class WristState {

    public final double angle, velocity;

    public WristState(double angle, double velocity) {
        this.angle = angle;
        this.velocity = velocity;
    }
    
    public String toString() {
        String output = "Wrist State(angle: "+angle+", Velocity: "+velocity+")";
        return output;
    }
}