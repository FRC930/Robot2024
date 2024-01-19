package frc.robot.utilities.constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class MotorConstants {
    private static MotorConstants instance;

    public final int KRAKEN_MAX_VOLTAGE = 24;

    private MotorConstants() {}

    public static MotorConstants getInstance() {
        if(instance == null) {
            instance = new MotorConstants();
        }
        return instance;
    }

    /**
     * <h3>resetTalonFX</h3>
     * Resets the talonFX to factory defaults
     * <p>This shortens a convoluted and annoying reset process
     * <p>{@link https://pro.docs.ctr-electronics.com/en/latest/docs/api-reference/api-usage/configuration.html#factory-default}
     * @param talon
     * @param brakeMode Sets whether the motor defaults to braking or coasting.
     */
    public static void resetTalonFX(TalonFX talon, boolean brakeMode) {
        talon.getConfigurator().apply(new TalonFXConfiguration());
        talon.setNeutralMode(brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }
}
