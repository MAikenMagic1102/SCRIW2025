package frc.robot.subsystems.Pivot;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;


public class Pivot {
    
public double homeScoreAngle = 0;
public double groundIntakeAngle = 0;
public double reefIntakeAngle = 0;
final TalonFX motorPivot = new TalonFX(7, "rio");
final DutyCycleOut motorPivotRequest = new DutyCycleOut(0.0);

StatusSignal<Angle> rotorPosSignal = motorPivot.getRotorPosition();
Angle rotorPos = rotorPosSignal.getValue();

public double currentAngle = rotorPosSignal.getValueAsDouble();

    public void groundIntake() {
        while (currentAngle >= 211 && currentAngle <= 209) {
            motorPivot.setControl(motorPivotRequest.withOutput(1));
        }
    }
    
    public void reefIntake() {
        while (currentAngle >= 121 && currentAngle <= 119) {
            motorPivot.setControl(motorPivotRequest.withOutput(1));
        }
    }

    public void homeScore() {
        while (currentAngle >= 1 && currentAngle <= -1) {
            motorPivot.setControl(motorPivotRequest.withOutput(1));
        }
    }
   
};
