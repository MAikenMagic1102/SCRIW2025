package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;



public class Climber {
    //TODO: Write climber logic. I could use some help, since I have no Idea what I'm doing. Thanks, Codecademy
    //Maybe I can look through the other code as reference...
final TalonFX motorClimber = new TalonFX(0, "rio");
final DutyCycleOut motorPivotRequest = new DutyCycleOut(0.0);
StatusSignal<Angle> rotorPosSignal = motorClimber.getRotorPosition();
Angle rotorPos = rotorPosSignal.getValue();
public double currentAngle = rotorPosSignal.getValueAsDouble();
//I don't know what I'm doing but I'm doing something :3
public void neutralPos() {

}
public void extendedPos() {
    
}
}
