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

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
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
  // private TalonFXSimState motorSim;
  // private CANcoderSimState cancoderSim;

  private static CANcoder pivotCaNcoder;

  // private CANcoderSimState cancoderSim;

  // private CANcoder armCaNcoder;

  private DCMotor pivotGearbox = DCMotor.getKrakenX60(1);

      armGearbox,
      PivotConstants.pivotGearing,
      SingleJointedArmSim.estimateMOI(PivotConstants.pivotLength, PivotConstants.pivotMass),
      PivotConstants.pivotLength,
      PivotConstants.pivotMinAngle,
      PivotConstants.pivotMaxAngle,
      true,
      PivotConstants.pivotStartingAngle);
    // private SingleJointeSim armSim = 
    //  new SingleJointedArmSim(
    //   armGearbox,
    //   PivotConstants.pivotGearing,
    //   SingleJointedArmSim.estimateMOI(PivotConstants.pivotLength, PivotConstants.pivotMass),
    //   PivotConstants.pivotLength,
    //   PivotConstants.pivotMinAngle,
    //   PivotConstants.pivotMaxAngle,
    //   true,
    //   PivotConstants.pivotStartingAngle);


  private DutyCycleOut dutyOut = new DutyCycleOut(0);
  private PositionVoltage posVoltage = new PositionVoltage(0).withSlot(0);
  private MotionMagicVoltage mmPosition = new MotionMagicVoltage(0).withSlot(0);

  boolean closedLoop = false;

  private double targetPosition = 0;

  // AdvantageScope log paths
  private final String loggerPath = "Subsystems/Pivot";
  private final String motorLoggerPath = loggerPath + "/Motor";
/** Creates a new Arm. */
  public Pivot() {
    // armCaNcoder = new CANcoder(PivotConstants.cancoderID, PivotConstants.busname);

    armMotor = new TalonFX(PivotConstants.motorID, PivotConstants.busname);
    // armCaNcoder = new CANcoder(PivotConstants.cancoderID, PivotConstants.busname);


    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = pivotMotor.getConfigurator().apply(PivotConstants.config);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    // motorSim = pivotMotor.getSimState();
    // cancoderSim = armCaNcoder.getSimState();
    // armCaNcoder.setPosition(0);

    // /* Retry config apply up to 5 times, report if failure */
    // StatusCode ccstatus = StatusCode.StatusCodeNotInitialized;
    // for (int i = 0; i < 5; ++i) {
    //   ccstatus = armCaNcoder.getConfigurator().apply(PivotConstants.ccconfig);
    //   if (ccstatus.isOK()) break;
    // }
    // if(!ccstatus.isOK()) {
    //   System.out.println("Could not apply configs, error code: " + ccstatus.toString());
    // }

    //armMotor.setPosition(armCaNcoder.getAbsolutePosition().getValueAsDouble() / ArmConstants.armRotorToSensor);
    motorSim = armMotor.getSimState();
    // cancoderSim = armCaNcoder.getSimState();
    // armCaNcoder.setPosition(0);

  }

  @Override
  public void periodic() {

    // if(armAtScoring()){
    //   PivotConstants.driveSpeed = 0.15;
    // }else{
    //   PivotConstants.driveSpeed = 1.0;
    // }
    // This method will be called once per scheduler run


    // Logging
    Logger.recordOutput(loggerPath + "/Angle", getAngleDegrees());
    Logger.recordOutput(loggerPath + "/At Goal", atGoal());

    Logger.recordOutput(motorLoggerPath + "/Motor Voltage", pivotMotor.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput(motorLoggerPath + "/Stator Current", pivotMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput(motorLoggerPath + "/Motor Temp", pivotMotor.getDeviceTemp().getValueAsDouble());

    SmartDashboard.putNumber("Pivot Angle", getAngleDegrees());
    
    if(closedLoop){
      Logger.recordOutput("Arm/ Setpoint", pivotMotor.getClosedLoopReference().getValueAsDouble());
    }
  }

  public boolean atGoal() {
    return Math.abs(targetPosition - getPositionMeters()) < PivotConstants.positionTolerence;
  }

    // cancoderSim.setRawPosition(Units.radiansToRotations(armSim.getAngleRads()) * PivotConstants.armGearingCANcoder);
  }

  public double getAngleDegrees(){
    // return Units.rotationsToDegrees(armCaNcoder.getPosition().getValueAsDouble() / PivotConstants.armGearingCANcoder);
  }

  public boolean pivotAtScoring(){
    return getAngleDegrees() < -180;
  }

  public boolean pivotAtHome(){
    Pivot.getAngleDegrees();
    return getAngleDegrees() > -8 && getAngleDegrees() < 8;

  public boolean reefLower(){
    return getAngleDegrees() > 0.24;
  }

  public boolean reefUpper(){
    return getAngleDegrees() > 0.24;
  }

  
  public boolean processor(){
    return getAngleDegrees() > 0.24;

  }

    public Command pivotAtHome_CMD(){
        return Commands.run(() ->pivotAtHome());
    }
  
  public boolean bargeScore(){
    return getAngleDegrees() > 0.24;
  }




  public void setPositionMeters(double height) {
    targetPosition = height;
    posVoltage.withPosition(height / PivotConstants.pivotGearing).withEnableFOC(true);
    pivotMotor.setControl(posVoltage);
  }
 
  public void setPositionMetersMM(double height) {
    targetPosition = height;
    mmPosition.withPosition(height / PivotConstants.pivotGearing).withEnableFOC(true);
    pivotMotor.setControl(mmPosition);
  }


  // public boolean pivotHalfScored(){
  //   return getAngleDegrees() < -100;
  // }

  // @Override
  // public void simulationPeriodic() {
  //   // This method will be called once per scheduler run
  //   motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
  //   // cancoderSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    
    // armSim.setInput(motorSim.getMotorVoltage());

    // armSim.update(0.020);

  //   RoboRioSim.setVInVoltage(
  //   //     BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));

  //   // motorSim.setRawRotorPosition(Units.radiansToRotations(armSim.getAngleRads()) * PivotConstants.pivotGearing);
  //   // cancoderSim.setRawPosition(Units.radiansToRotations(armSim.getAngleRads()) * PivotConstants.armGearingCANcoder);
  // }

  public double getAngleDegrees(){
    return Units.rotationsToDegrees(pivotMotor.getPosition().getValueAsDouble());
  }

  // public boolean armAtScoring(){
  //   return getAngleDegrees() < -180;
  // }

  // public boolean armAtHome(){
  //   return getAngleDegrees() > -8 && getAngleDegrees() < 8;
  // }
  
  // public boolean atGoal(){
  //   return Math.abs(targetPosition - getAngleDegrees()) < PivotConstants.positionTolerence;
  // }

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
  public double getPositionMeters() {
    return rotationsToMeters(pivotMotor.getRotorPosition().getValue()).in(Meters);
  }

  
  public static Distance rotationsToMeters(Angle rotations) {
    /* Apply gear ratio to input rotations */
    var gearedRadians = rotations.in(Radians) / ElevatorConstants.elevatorGearing;
    /* Then multiply the wheel radius by radians of rotation to get distance */
    return ElevatorConstants.elevatorPullyRadiusDistance.times(gearedRadians);
  }
}