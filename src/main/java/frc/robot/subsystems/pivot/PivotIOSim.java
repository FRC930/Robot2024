package frc.robot.subsystems.pivot;

public class PivotIOSim implements PivotIO {

    private static final double GEAR_RATIO = 0; // TODO: Add from E-Mech


    //private final SingleJointedArmSim sim = new SingleJointedArmSim(DCMotor.getNEO(1), 75,
    //SingleJointedArmSim.estimateMOI(Units.inchesToMeters(24.719), Units.lbsToKilograms(11)),
    //Units.inchesToMeters(24.719), -2 * Math.PI, 2 * Math.PI, true);

    @Override
    public void updateInputs() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
    }

    @Override
    public double getCurrentAngleDegrees() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getCurrentAngleDegrees'");
    }

    @Override
    public double getVelocityDegreesPerSecond() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getVelocityDegreesPerSecond'");
    }

    @Override
    public void setVoltage(double volts) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setVoltage'");
    }

    @Override
    public void adjustOffsetDegrees(double offsetDegrees) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'adjustOffsetDegrees'");
    }
    
}
