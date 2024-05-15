package frc.robot.IOs;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.hardware.TalonFX;

public interface TalonVelocityIO {
    /**
     * <h3>runSim</h3>
     * Steps the simulation forward by 0.02 seconds.
     * <p>Does nothing on physical robot
     */
    public void runSim();
    /**
    * <h3>setSpeed</h3>
    * sets the speed with slot 0
    * @param speed the speed the wheel will be set to
    */
    void setSpeed(double speed, double acceleration);
    /**
    * <h3>setSpeed</h3>
    * sets the speed
    * @param speed the speed the wheel will be set to
    * @param acceleration the acceleraction lol
    */
    void setSpeedWithSlot(double speed, double acceleration, int slot);
    /**
    * <h3>setSpeed</h3>
    * sets the speed with slot 0
    * @param speed the speed the wheel will be set to.
    */
    void setSpeed(double speed);
    /**
    * <h3>setSpeed</h3>
    * sets the speed
    * @param speed the speed the wheel will be set to
    * @param slot the slot
    */
    void setSpeedWithSlot(double speed, int slot);
    /**
    * <h3>geMotorSpeed</h3>
    * @return The current motor speed of the wheel in rps
    */
    double getSpeed();

    /**
    * <h3>getVoltage</h3>
    * @return The current voltage of the motor
    */
    double getVoltage();

    /**
    * <h3>getTargetVelocity</h3>
    * @return The current voltage of the right motor
    */
    double getTargetVelocity();
     /**
     * <h1>getTalon</h1> 
     * @return the talonFX controller of this roller.
     */
    public TalonFX getTalon();
    /**
    * <h3>stop</h3>
    * This sets the motor's speed to 0
    */
    void stop();
    /**
     * <h3>getPeriod</h3>
     * @return
     */
    default double getPeriod() {
        return 0.02;
    }

    public void setSlot(Slot0Configs config);
    public void setSlot(Slot1Configs config);
    public void setSlot(Slot2Configs config);
}