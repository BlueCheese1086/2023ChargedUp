package frc.robot.StateManager.Commands;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Sensors.Field.PositionManager;
import frc.robot.StateManager.StateManager;
import frc.robot.StateManager.StateManager.Positions;
import frc.robot.Util.BlueCheeseAlliance;

public class AutoEE extends CommandBase {

    private final StateManager state;

    private final PositionManager pManager;

    private final BooleanSupplier stowedOverrider;

    private final ArrayList<Positions> allowedPositions = new ArrayList<>();

    public AutoEE(StateManager state, BooleanSupplier stowedOverrider, IntSupplier level) {
        this.state = state;
        this.stowedOverrider = stowedOverrider;
        pManager = PositionManager.getInstance();
        addRequirements(state);
    }

    @Override
    public void execute() {
        allowedPositions.clear();
        BlueCheeseAlliance alliance = BlueCheeseAlliance.getAlliance();

        if (stowedOverrider.getAsBoolean()) {
            state.setPosition(Positions.stowed);
            state.executePosition();
            return;
        }
        
        if (pManager.getCurrentLocation().equals(alliance.scoring)) {
            allowedPositions.addAll(List.of(Positions.ground, Positions.mid, Positions.high));
        } else if (pManager.getCurrentLocation().equals(alliance.pickup)) {
            allowedPositions.add(Positions.player);
        }

        if (allowedPositions.size() == 0) {
            state.setPosition(Positions.stowed);
        } else {
            state.setPosition(Positions.high);
        }

        state.executePosition();

    }
    
}
