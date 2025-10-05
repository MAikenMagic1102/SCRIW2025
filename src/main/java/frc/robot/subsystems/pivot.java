package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;


public class Pivot {
public double homeScoreAngle = 0;
public double groundIntakeAngle = 0;
public double reefIntakeAngle = 0;
final TalonFX motorPivot = new TalonFX(7, "rio");
final DutyCycleOut motorPivotRequest = new DutyCycleOut(0.0);

StatusSignal<Angle> rotorPosSignal = motorPivot.getRotorPosition();
Angle rotorPos = rotorPosSignal.getValue();

public double currentAngle = rotorPosSignal.getValueAsDouble();

    public Command groundIntake() {
        while (currentAngle >= 211 && currentAngle <= 209) {
            return Commands.runOnce(()-> motorPivot.setControl(motorPivotRequest.withOutput(1)));
        }
        return Commands.none();
    }
    
    public Command reefIntake() {
        while (currentAngle >= 121 && currentAngle <= 119) {
            return Commands.runOnce(()-> motorPivot.setControl(motorPivotRequest.withOutput(1)));
        }
        return Commands.none();  // does nothing; satisfies the compiler
    }

    public Command homeScore() {
        while (currentAngle >= 1 && currentAngle <= -1) {
            return Commands.runOnce(()-> motorPivot.setControl(motorPivotRequest.withOutput(1)));
        }
        return Commands.none();
    }
   
};
