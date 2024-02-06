package frc.robot.utilities;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;

public final class Phoenix6Utility {
    
    private static final int CONFIG_RETRY_COUNT = 5;

    /**
     * Applies a blank Talon Configuration to the given talon, in order to factory reset
     * @param talon talon to factory reset
     * @return
     */
    public static StatusCode resetTalonFxFactoryDefaults(TalonFX talon) {
        return setTalonFxConfiguration(talon, new TalonFXConfiguration());
    }
    
    /**
     * Applies cfg to talon with retry
     * @param talon talon to apply config to
     * @param cfg Talon configuration to apply
     * @return
     */
    public static StatusCode setTalonFxConfiguration(TalonFX talon, TalonFXConfiguration cfg) {
        return applyConfigAndRetry(talon, () -> talon.getConfigurator().apply(cfg));
    }

    /**
     * Apply Configure to Ctre device and validate configuration is applied (retry if fails)
     * @param device the device to apply to (ie talon/CANcoder)
     * @param toApply Supplier of method that returns a StatusCode
     */
    public static StatusCode applyConfigAndRetry(ParentDevice device, Supplier<StatusCode> toApply) {
        return applyConfig(device, toApply, false);
    }

    /**
     * Apply Configure to Ctre device and validate configuration is applied (no retry if fails)
     * @param device the device to apply to (ie talon/CANcoder)
     * @param toApply Supplier of method that returns a StatusCode
     */
    public static StatusCode applyConfigAndNoRetry(ParentDevice device, Supplier<StatusCode> toApply) {
        return applyConfig(device, toApply, true);
    }


    private static StatusCode applyConfig(ParentDevice device, Supplier<StatusCode> toApply, boolean noRetry) {
        StatusCode finalCode = StatusCode.StatusCodeNotInitialized;
        int triesLeftOver = noRetry ? 1 : CONFIG_RETRY_COUNT;
        do{
            finalCode = toApply.get();
        } while (!finalCode.isOK() && --triesLeftOver > 0);
        if(finalCode.isOK()) {
            DriverStation.reportError(
                String.format("Unable to configure device %s: %s", device.getDeviceID(), finalCode.toString()), 
                true);
        }
        return finalCode;
    }

}
