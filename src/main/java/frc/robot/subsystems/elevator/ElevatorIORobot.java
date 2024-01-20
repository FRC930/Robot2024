package frc.robot.subsystems.elevator;

import frc.robot.utilities.TalonWrapper;

public class ElevatorIORobot implements ElevatorIO {
    private TalonWrapper leftElevatorFollower;
    private TalonWrapper rightElevatorMaster;
    private double circumference;
    private double gearRatio;

    /**
     * <h3>ElevatorIORobot</h3> 
     * Creates a subsystem that represents an Elevator system
     * @param motorID The id of the elevator motor
     */
    public ElevatorIORobot (TalonWrapper motor1, TalonWrapper motor2, double circumference, double gearRatio){
        leftElevatorFollower = motor1;
        rightElevatorMaster = motor2;

        leftElevatorFollower.resetToFactoryDefaults();
        leftElevatorFollower.setShouldBrake(true);
        rightElevatorMaster.resetToFactoryDefaults();
        rightElevatorMaster.setShouldBrake(true);

        leftElevatorFollower.followIO(rightElevatorMaster,true);
        this.circumference = circumference;
        this.gearRatio = gearRatio;
    }

    @Override
    public void updateInputs() {}

    @Override
    public void setVoltage(double volts) {
        rightElevatorMaster.setVoltage(volts);
    }

    @Override
    public double getCurrentVelocity() {
        return rightElevatorMaster.getVelocity().getValue();
    }

    @Override
    public double getCurrentHeight() {
        return rightElevatorMaster.getPosition().getValue() * circumference/gearRatio;
    }  
}
