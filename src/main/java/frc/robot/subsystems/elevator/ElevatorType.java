package frc.robot.subsystems.elevator;

public enum ElevatorType {
    SHOOTING_ELEVATOR(1,1,1,0,0.509,0), //TODO: Configure constants
    CLIMBING_ELEVATOR(1,1,1,0,0.509,0); //TODO: Configure constants

    public final double m_gearRatio;
    public final double m_kV;
    public final double m_kA;
    public final double m_minHeight;
    public final double m_maxHeight;
    public final double m_startingHeight;

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
