package frc.robot.subsystems.Elevator;


public class ElevatorConstants {

    public static final int m_motorLeft_ID = 20;
    public static final int m_motorRight_ID = 21;

//A

public static final double cmPerRot = 0;
public static final double cmElevatorZero = 0;

public static final double errorRange = 0.05;


public static final class ElevatorSetpoints {
    public static final double barge = 0;
    public static final double processor = 0;
    public static final double elevatorGroundIntake  = 0;
    public static final double elevatorLowerReef = 0;
    public static final double elevatorUpperReef = 0;
    public static final double elevatorHome = 0;
}

public static final class SimulationRobotConstants {

    public static final double MinElevatorHeightMeters = 0; // m
    public static final double MaxElevatorHeightMeters = 0; // m


}

}


//A
// private final CANcoder leftSensor = new CANcoder(22);
// private final CANcoder rightSensor = new CANcoder(23);

// final DutyCycleOut m_dutyMotorLeft = new DutyCycleOut(0.0);
// final DutyCycleOut m_dutyMotorRight = new DutyCycleOut(0.0);

// private final TalonFXSimState leftSim = m_motorLeft.getSimState();
// private final TalonFXSimState rightSim = m_motorRight.getSimState();
