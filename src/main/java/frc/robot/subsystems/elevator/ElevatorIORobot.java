package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

public class ElevatorIORobot implements ElevatorIO {
    private TalonFX leftElevatorFollower;
    private TalonFX rightElevatorMaster;
    private double circumference;
    private double gearRatio;

    /**
     * <h3>ElevatorIORobot</h3> 
     * Creates a subsystem that represents an Elevator system
     * @param motorID The id of the elevator motor
     */
    public ElevatorIORobot (int rightMotorID, int leftMotorID, double circumference, double gearRatio){
        leftElevatorFollower = new TalonFX(leftMotorID);
        rightElevatorMaster = new TalonFX(rightMotorID);
        leftElevatorFollower.setControl(new Follower(leftMotorID, true));
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
