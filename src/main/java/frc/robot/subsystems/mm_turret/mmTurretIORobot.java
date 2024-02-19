package frc.robot.subsystems.mm_turret;


import java.util.Map;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.IOs.TalonPosIO;
import frc.robot.utilities.CommandFactoryUtility;
import frc.robot.utilities.Phoenix6Utility;


/**
 * <h3>PivotIORobot</h3>
 * An IO to the real robot's pivot
 */
public class mmTurretIORobot implements TalonPosIO{
    protected TalonFX m_motor;

    private MotionMagicVoltage m_request;
    private DutyCycleEncoder m_encoder;
    /**
     * <h3>PivotIORobot</h3> 
     * Creates a subsystem that represents the actual pivot subsystem
     */
    public mmTurretIORobot(int id, int encoderID, String canbus, double gearRatio, Slot0Configs config, MotionMagicConfigs mmConfigs, double encoderOffset) {
        m_motor = new TalonFX(id, canbus);
        m_encoder = new DutyCycleEncoder(encoderID);

        m_request = new MotionMagicVoltage(0);

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.withSlot0(config.withGravityType(GravityTypeValue.Elevator_Static)); // PID/FF configs
        cfg.withMotionMagic(mmConfigs); // Motion magic configs
        cfg.Feedback.SensorToMechanismRatio = gearRatio; // Applies gear ratio
        
        
        Phoenix6Utility.setTalonFxConfiguration(m_motor, cfg);
        m_motor.setNeutralMode(NeutralModeValue.Brake); // Enables brake mode
        m_motor.setInverted(false); //with metal gears it was not inverted 

        // Zeros motor encoder using through bore
        m_encoder.setPositionOffset(Units.degreesToRotations(encoderOffset));

        // if(!Robot.isReal()) {
        //     configure();
        // } // else disableInit() will run the configuration (delaying so bore encoder can has time to start up)
    }

    @Override
    public void configure() {
        // TODO WHAT IF ENCODER DISCONNECTED OR BROKEN (INFINITE LOOP!!!!)
        try {
            while(!m_encoder.isConnected()) {
                Thread.sleep(10);
                System.out.println("****************************DUTY CYCLE ENCODER NOT RUNNING******************************************************************************");
            } 
            Thread.sleep(50);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        double absolutePos = m_encoder.getAbsolutePosition();
        double offset = m_encoder.getPositionOffset();

        Logger.recordOutput("mmTurretSubsystem/AbsolutePosition",
                Units.rotationsToDegrees(absolutePos));
        Logger.recordOutput("mmTurretSubsystem/Offset",
                Units.rotationsToDegrees(offset));

        // The same
        // double position = Units.degreesToRotations(Math.IEEEremainder(Units.rotationsToDegrees(absolutePos - offset),360.0));
        double position = Math.IEEEremainder(absolutePos - offset, 1.0);
        
        // double position = 
        //     (absolutePos - offset > 0.5) // Change negative values to wrap around back to 0.0-1.0
        //         ? absolutePos - offset - 1.0
        //         : absolutePos - offset;




        Phoenix6Utility.applyConfigAndRetry(m_motor,
            () -> m_motor.getConfigurator().setPosition(position));
            
        // Move to stow pos
        Phoenix6Utility.applyConfigAndRetry(m_motor,
            () -> m_motor.setControl(m_request.withPosition(Units.degreesToRotations(CommandFactoryUtility.TURRET_STOW_POS)).withSlot(0)));
    }
    
    @Override
    public void runSim() {}

    @Override
    public double getPos() {
        // TODO remove logging that is not needed
        Logger.recordOutput("mmTurretSubsystem/ChangedAbsoluteDegrees", Units.rotationsToDegrees(m_encoder.getAbsolutePosition() - m_encoder.getPositionOffset()));
        Logger.recordOutput("mmTurretSubsystem/MathedAbsoluteDegrees", Math.IEEEremainder(Units.rotationsToDegrees(m_encoder.getAbsolutePosition() - m_encoder.getPositionOffset()), 360.0));
        Logger.recordOutput("mmTurretSubsystem/InitialAbsoluteDegrees", Units.rotationsToDegrees(m_encoder.getAbsolutePosition()));

        return Units.rotationsToDegrees(m_motor.getPosition().getValue());
    }

    @Override
    public double getVelocity() {
       return Units.rotationsToDegrees(m_motor.getVelocity().getValue());
    }

    @Override
    public void setTarget(double position) {
        Phoenix6Utility.applyConfigAndNoRetry(m_motor,
            () -> m_motor.setControl(m_request.withPosition(Units.degreesToRotations(position)).withSlot(0)));
    }

    @Override
    public double getVoltage() {
        return m_motor.getMotorVoltage().getValue();
    }

    @Override
    public double getTarget() {
        // Position configuration may not be available yet, so allow for Position not being available yet
        Map<String, String> map = m_motor.getAppliedControl().getControlInfo();
        String position = map.get("Position");
        if(position == null) {
            position = "0.0";
        }
        return Units.rotationsToDegrees(Double.valueOf(position));
    }
}
