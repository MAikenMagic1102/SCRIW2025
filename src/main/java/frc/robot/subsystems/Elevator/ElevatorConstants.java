package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.controls.DutyCycleOut;


public class ElevatorConstants {

final TalonFX m_motorLeft = new TalonFX(0);
final TalonFX m_motorRight = new TalonFX(1);
private final CANcoder leftSensor = new CANcoder(0);
private final CANcoder rightSensor = new CANcoder(1);

final DutyCycleOut m_motorleft = new DutyCycleOut(0.0);
final DutyCycleOut m_motorright = new DutyCycleOut(0.0);

private final TalonFXSimState leftSim = m_motorLeft.getSimState();
private final TalonFXSimState rightSim = m_motorRight.getSimState();

public static final double cmPerRot = 0;
public static final double cmElevatorZero = 0;

public static final class ElevatorSetpoints {
    public static final double barge = 0;
    public static final double processor = 0;
    public static final double elevatorGroundIntake  = 0;
    public static final double elevatorLowerReef = 0;
    public static final double elevatorUpperReef = 0;
    public static final double elevatorHome = 0;
    public static final double elevatorscore = 0;
}

public static final class SimulationRobotConstants {

    public static final double MinElevatorHeightMeters = 0; // m
    public static final double MaxElevatorHeightMeters = 0; // m


}


// private final CANcoderSimState leftSensSim = m_motorLeft.getSimState();
// private final CANcoderSimState rightSensSim = m_motorRight.getSimState();


    // private final Pigeon2 imu = new Pigeon2(0);

    // private final TalonFXSimState leftSim = leftFX.getSimState();
    // private final TalonFXSimState rightSim = rightFX.getSimState();
    // private final CANcoderSimState leftSensSim = leftSensor.getSimState();
    // private final CANcoderSimState rightSensSim = rightSensor.getSimState();

}
