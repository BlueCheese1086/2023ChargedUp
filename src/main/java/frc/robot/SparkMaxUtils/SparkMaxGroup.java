package frc.robot.SparkMaxUtils;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SparkMaxGroup extends SubsystemBase {

    private SparkMax lead;
    private final ArrayList<SparkMax> maxes = new ArrayList<>();
    private final ArrayList<SparkMax> dead = new ArrayList<>();

    public SparkMaxGroup(SparkMax lead, SparkMax... maxes) {
        this.lead = lead;
        this.maxes.addAll(Arrays.asList(maxes));

        for (SparkMax max : maxes) {
            max.follow(lead, max.getConfig().INVERTED);
        }
    }

    public void periodic() {
        if (!lead.isConnected()) {
            if (maxes.size() > 0) {
                dead.add(lead);
                lead = maxes.remove(0);
            }
        }
    }

    public SparkMax getLead() {
        return lead;
    }

    public void attemptRecovery() {
        for (int i = 0; i < dead.size(); i++) {
            if (dead.get(i).isConnected()) {
                maxes.add(dead.remove(i));
                i--;
            }
        }
    }
    
}
