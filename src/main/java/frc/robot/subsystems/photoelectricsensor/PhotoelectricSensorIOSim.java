package frc.robot.subsystems.photoelectricsensor;

import frc.robot.subsystems.algaeclaw.AlgaeClawConstants;
import frc.robot.subsystems.coralholder.CoralHolderConstants;
import frc.robot.subsystems.gamepiecevisualizers.AlgaeVisualizer;
import frc.robot.subsystems.gamepiecevisualizers.CoralVisualizer;

public class PhotoelectricSensorIOSim implements PhotoelectricSensorIO {
    private final String id;

    public PhotoelectricSensorIOSim(String id) {
        this.id = id;
    }

    @Override
    public void updateInputs(PhotoelectricSensorIOInputs inputs) {
        if (id.equals(CoralHolderConstants.coralHolderPEId)) {
            inputs.voltage =
                    switch (CoralVisualizer.coralState) {
                        case LOADED -> 0.0;
                        case GONE -> 5.0;
                        default -> 5.0;
                    };
        } else if (id.equals(AlgaeClawConstants.algaeClawPEID)) {
            inputs.voltage =
                    switch (AlgaeVisualizer.algaeState) {
                        case LOADED -> 0.0;
                        case GONE -> 5.0;
                        default -> 5.0;
                    };
        } else {
            inputs.voltage = 5.0;
        }
    }
}
