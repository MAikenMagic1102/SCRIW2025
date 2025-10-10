// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Pivot;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator.ElevatorConstants;

public class Pivot extends SubsystemBase {
  private TalonFX pivotMotor;
  private TalonFXSimState motorSim;
  // private CANcoderSimState cancoderSim;

  // private static CANcoder pivotCaNcoder;

  private DCMotor pivotGearbox = DCMotor.getKrakenX60(1);

  // private SingleJointedArmSim armSim = 
  //   new SingleJointedArmSim(
  //     pivotGearbox,
  //     PivotConstants.pivotGearing,
  //     SingleJointedArmSim.estimateMOI(PivotConstants.pivotLength, PivotConstants.pivotMass),
  //     PivotConstants.pivotLength,
  //     PivotConstants.pivotMinAngle,
  //     PivotConstants.pivotMaxAngle,
  //     true,
  //     PivotConstants.pivotStartingAngle);

  private DutyCycleOut dutyOut = new DutyCycleOut(0);
  private PositionVoltage posVoltage = new PositionVoltage(0).withSlot(0);
  private MotionMagicVoltage mmVoltage = new MotionMagicVoltage(0).withSlot(0);

  boolean closedLoop = false;

  private double targetPosition = 0;

  // AdvantageScope log paths
  private final String loggerPath = "Subsystems/Pivot";
  private final String motorLoggerPath = loggerPath + "/Motor";
/** Creates a new Arm. */
  public Pivot() {
    pivotMotor = new TalonFX(PivotConstants.motorID, PivotConstants.busname);
    // pivotCaNcoder = new CANcoder(PivotConstants.cancoderID, PivotConstants.busname);

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = pivotMotor.getConfigurator().apply(PivotConstants.config);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    // /* Retry config apply up to 5 times, report if failure */
    // StatusCode ccstatus = StatusCode.StatusCodeNotInitialized;
    // for (int i = 0; i < 5; ++i) {
    //   ccstatus = pivotCaNcoder.getConfigurator().apply(PivotConstants.ccconfig);
    //   if (ccstatus.isOK()) break;
    // }
    // if(!ccstatus.isOK()) {
    //   System.out.println("Could not apply configs, error code: " + ccstatus.toString());
    // }

    //armMotor.setPosition(armCaNcoder.getAbsolutePosition().getValueAsDouble() / ArmConstants.armRotorToSensor);
    // motorSim = pivotMotor.getSimState();
    // cancoderSim = pivotCaNcoder.getSimState();
    // pivotCaNcoder.setPosition(0);
  }

  @Override
  public void periodic() {


    // Logging
    // Logger.recordOutput(loggerPath + "/Angle", getAngleDegrees());
    // Logger.recordOutput(loggerPath + "/At Goal", atGoal());

    Logger.recordOutput(motorLoggerPath + "/Motor Voltage", pivotMotor.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput(motorLoggerPath + "/Stator Current", pivotMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput(motorLoggerPath + "/Motor Temp", pivotMotor.getDeviceTemp().getValueAsDouble());
    
    if(closedLoop){
      Logger.recordOutput("Arm/ Setpoint", pivotMotor.getClosedLoopReference().getValueAsDouble());
    }
  }

  public void setOpenLoop(double demand){
    dutyOut.withOutput(demand);
    pivotMotor.setControl(dutyOut);
    closedLoop = false;
  }

  public void setAnglePosition(double angle){
    targetPosition = angle;
    posVoltage.withPosition(Units.degreesToRotations(angle));
    pivotMotor.setControl(posVoltage);
    closedLoop = true;
  }
  

  public Command setAngle(double angle){
    return runOnce(() -> setAnglePosition(angle));
  }

  public Command  setAngleCommand(double angle){
    return runOnce(() -> setAngle(angle));
  }
  

}