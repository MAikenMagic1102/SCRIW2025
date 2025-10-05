// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  private final TalonFX motorClimber;

  private final DutyCycleOut motorPivotRequest = new DutyCycleOut(0.0);
  private StatusSignal<Angle> rotorPosSignal;

  private Angle rotorPos;
  private double currentAngle;

  /** Creates a new Climber. */
  public Climber() {
    motorClimber = new TalonFX(50, "rio");

    TalonFXConfiguration motor_Config = new TalonFXConfiguration();

    //This has to be tuned for position control
    motor_Config.Slot0.kP = 0;
    //Set this correctly to make up positive and down negative
    motor_Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motor_Config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motorClimber.getConfigurator().apply(motor_Config);

    rotorPosSignal = motorClimber.getRotorPosition();

    rotorPos = rotorPosSignal.getValue();
    currentAngle = rotorPosSignal.getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    rotorPosSignal = motorClimber.getRotorPosition();
    
    rotorPos = rotorPosSignal.getValue();
    currentAngle = rotorPosSignal.getValueAsDouble();
  }
}
