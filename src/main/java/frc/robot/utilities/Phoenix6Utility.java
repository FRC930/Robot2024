package frc.robot.utilities;

import java.util.Map;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Provided by Steve Wegner per Phoenix6 example, modified by Harry Prochnow
 */
public final class Phoenix6Utility {
    
    private static final int CONFIG_RETRY_COUNT = 5;

    /**
     * Applies a blank Talon Configuration to the given talon, in order to factory reset
     * @param talon talon to factory reset
     * @return
     */
    public static StatusCode resetTalonFxFactoryDefaults(TalonFX talon) {
        return setTalonFxConfiguration(talon, new TalonFXConfiguration(), false);
    }

    /**
     * Applies cfg to talon with retry
     * @param talon talon to apply config to
     * @param cfg Talon configuration to apply
     * @return
     */
    public static StatusCode setTalonFxConfiguration(TalonFX talon, TalonFXConfiguration cfg) {
        return setTalonFxConfiguration(talon, cfg, true);
    }
    
    /**
     * Applies cfg to talon with retry
     * @param talon talon to apply config to
     * @param cfg Talon configuration to apply
     * @param resetFacDefs whether to reset factory defaults or not
     * @return
     */
    public static StatusCode setTalonFxConfiguration(TalonFX talon, TalonFXConfiguration cfg, boolean resetFacDefs) {
        if (resetFacDefs) {
            resetTalonFxFactoryDefaults(talon);
        }
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
        if(!finalCode.isOK()) {
            DriverStation.reportWarning(
                String.format("Unable to configure device %s: %s", device.getDeviceID(), finalCode.toString()), 
                true);
        }
        return finalCode;
    }

    /**
     * Get position from controller so dont have to explicitly cast applied controller (could have had runtime casting error when switch controllers)
     * @param motor
     * @param defaultPosition
     * @return
     */
    public static double getPositionFromController(ParentDevice motor, double defaultPosition) {
        // Position configuration may not be available yet, so allow for Position not being available yet
        Map<String, String> map = motor.getAppliedControl().getControlInfo();
        String positionString = map.get("Position");
        double position= defaultPosition; // If controller is not available delayed configurations
        if(positionString != null) {
            position = Double.valueOf(positionString);
        }
        return position;
        // This is to eliminate calls like this and possible casting errors when switch controllers
        // return (MotionMagicExpoVoltage) rightElevatorMaster.getAppliedControl()).Position
    }

    /**
     * Get velocity from controller so dont have to explicitly cast applied controller (could have had runtime casting error when switch controllers)
     * @param motor
     * @param defaultVelocity
     * @return
     */
    public static double getVelocityFromController(ParentDevice motor, double defaultVelocity) {
        // Velocity configuration may not be available yet, so allow for Velocity not being available yet
        Map<String, String> map = motor.getAppliedControl().getControlInfo();
        String velocityString = map.get("Velocity");
        double velocity= defaultVelocity; // If controller is not available delayed configurations
        if(velocityString != null) {
            velocity = Double.valueOf(velocityString);
        }
        return velocity;
        // This is to eliminate calls like this and possible casting errors when switch controllers
        // return ((VelocityTorqueCurrentFOC) m_motor.getAppliedControl()).Velocity;
    }

    // public static TalonFX setTalonCurrentLimits(TalonFX talon, double statorMax, double supplyMax) {
    //     return applyConfigAndRetry(talon, ()-> {return new CurrentLimitsConfigs().withStatorCurrentLimit(statorMax).withSupplyCurrentLimit(supplyMax)});
    // }

}
