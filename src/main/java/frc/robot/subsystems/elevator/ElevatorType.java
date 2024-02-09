package frc.robot.subsystems.elevator;

public enum ElevatorType {
    //1 divided by diameter * Pi * gear ratio <2>. TODO test to see if closer to an inch   
    //1 divided by diameter * Pi * (1/gear ratio <2>). Pretty close to an inch
    SHOOTING_ELEVATOR(1.0 / (1.253 * Math.PI * (1.0/2.0)),1,1,0,12,0), //TODO: Configure constants
    CLIMBING_ELEVATOR(1,1,1,0,5,0); //TODO: Configure constants

    public final double m_gearRatio;
    public final double m_kV;
    public final double m_kA;
    public final double m_minHeight;
    public final double m_maxHeight;
    public final double m_startingHeight;
    
    /**
     * 
     * @param gearRatio
     * @param kV
     * @param kA
     * @param minHeight inches
     * @param maxHeight inches
     * @param startingHeight inches
     */
    private ElevatorType(
        double gearRatio, 
        double kV,
        double kA,
        double minHeight,
        double maxHeight, 
        double startingHeight
        ) {
        m_gearRatio = gearRatio;
        m_kV = kV;
        m_kA = kA;
        m_maxHeight = maxHeight;
        m_minHeight = minHeight;
        m_startingHeight = startingHeight;  
    }
}
