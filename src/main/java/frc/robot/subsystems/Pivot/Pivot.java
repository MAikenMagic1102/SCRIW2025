package frc.robot.subsystems.Pivot;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;


public class Pivot extends SubsystemBase{ 

    private final TalonFX motorPivot;
    private final DutyCycleOut motorPivotRequest = new DutyCycleOut(0.0);
    private final PositionVoltage motorPositionRequest = new PositionVoltage(0.0);

    private StatusSignal<Angle> rotorPosSignal;
    private Angle rotorPos;

    public double homeScoreAngle = 0;
    public double reefIntakeAngle = 0;
    public double currentAngle;

    public Pivot(){
      motorPivot = new TalonFX(PivotConstants.pivot_ID, "rio");
      TalonFXConfiguration motor_Config = new TalonFXConfiguration();
      
      //This has to be tuned for position control
      motor_Config.Slot0.kP = 0;
      //Set this correctly to make up positive and down negative
      motor_Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      motor_Config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      //Set this to gearing
      motor_Config.Feedback.SensorToMechanismRatio = 0.0;
      motorPivot.getConfigurator().apply(motor_Config);
      rotorPosSignal = motorPivot.getRotorPosition();
      rotorPos = rotorPosSignal.getValue();
      currentAngle = rotorPosSignal.getValueAsDouble();
    }

    public void setPosition(double position){
      motorPivot.setControl(motorPositionRequest.withPosition(position));
    }

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

    @Override
    public void periodic(){
        rotorPosSignal = motorPivot.getRotorPosition();
        rotorPos = rotorPosSignal.getValue();
        currentAngle = rotorPosSignal.getValueAsDouble();
    }
   
};
