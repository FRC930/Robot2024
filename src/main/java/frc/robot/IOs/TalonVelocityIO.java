package frc.robot.IOs;

public interface TalonVelocityIO {
    /**
     * <h3>runSim</h3>
     * Steps the simulation forward by 0.02 seconds.
     * <p>Does nothing on physical robot
     */
    public void runSim();
    /**
    * <h3>setSpeed</h3>
    * @param speed the speed the wheel will be set to
    */
    void setSpeed(double speed);

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
    * <h3>stop</h3>
    * This sets the shooter's speed to 0
    */
    void stop();
    /**
     * <h3>getPeriod</h3>
     * @return
     */
    default double getPeriod() {
        return 0.02;
    }
}