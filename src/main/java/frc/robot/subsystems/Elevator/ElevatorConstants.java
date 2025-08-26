package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class ElevatorConstants {

final TalonFX m_motorLeft = new TalonFX(0);
final TalonFX m_motorRight = new TalonFX(1);
private final CANcoder leftSensor = new CANcoder(0);
private final CANcoder rightSensor = new CANcoder(1);

private final TalonFXSimState leftSim = m_motorLeft.getSimState();
private final TalonFXSimState rightSim = m_motorRight.getSimState();






// private final CANcoderSimState leftSensSim = m_motorLeft.getSimState();
// private final CANcoderSimState rightSensSim = m_motorRight.getSimState();


    // private final Pigeon2 imu = new Pigeon2(0);

    // private final TalonFXSimState leftSim = leftFX.getSimState();
    // private final TalonFXSimState rightSim = rightFX.getSimState();
    // private final CANcoderSimState leftSensSim = leftSensor.getSimState();
    // private final CANcoderSimState rightSensSim = rightSensor.getSimState();

}
