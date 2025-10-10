// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superStructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Pivot.Pivot;
import frc.robot.subsystems.Pivot.PivotConstants;

public class SuperStructure extends SubsystemBase {

  private final Elevator elevator = new Elevator ();
  private final Pivot pivot = new Pivot();
  private final Intake intake = new Intake();
  private final int deafualtWait = 10;

  // Elevator Commands
  public Command setElevatorToScore(){
    return new InstantCommand(() -> elevator.setPositionMetersMM(ElevatorConstants.barge));
  }

  public Command setElevatorToScoreSafe(){
    return new InstantCommand(() -> elevator.setPositionMetersMM(ElevatorConstants.safePos));
  }

  public Command runElevatorUp() {
    return new InstantCommand(() -> elevator.setOpenLoop(0.25));
  }
  public Command runElevatorDown(){
    return new InstantCommand(() -> elevator.setOpenLoop(-0.25));
  }

  public Command stopElevator() {
    return new InstantCommand(() -> elevator.setOpenLoop(0.05));
  }

  // Pivot Commands
  public Command stopPivot() {
    return new InstantCommand(() -> pivot.setOpenLoop(-.020));
  }
  public Command goPivot() {
    return new InstantCommand(() -> pivot.setOpenLoop(-0.2));
  }
  public Command noPivot() {
    return new InstantCommand(() -> pivot.setOpenLoop(0.2));
  }
  public Command setPivotToScore() {
    return new InstantCommand(() -> pivot.setAnglePosition(PivotConstants.bargeScore));
  }
  public Command setPivotIntake() {
    return new InstantCommand(() -> pivot.setAnglePosition(PivotConstants.intakePos));
  }

  // Instake Comands
  public Command stopIntake() {
    return new InstantCommand(() -> intake.setOpenLoop(.05));
  }
  public Command outIntake() {
    return new InstantCommand(() -> intake.setOpenLoop(-.75));
  }
  public Command goIntake() {
    return new InstantCommand(() -> intake.setOpenLoop(.5));
  }

 // Home/Stored Commands
 public Command setElevatorHome(){
  return new InstantCommand(() -> elevator.setPositionMetersMM(0.007));
 }  

 public Command setPivotStored(){
  return new InstantCommand(() -> pivot.setAnglePosition(PivotConstants.stored));
 }

 // Multi Stage sequence commands
 public Command bargeScoreCommand = Commands.sequence(
  setPivotToScore(),
  setElevatorToScore(),
  new WaitCommand(deafualtWait),
  outIntake(),
  setElevatorHome(),
  setPivotStored()
 );

 public Command floorIntakeCommand = Commands.sequence(
  setElevatorHome(),
  setPivotIntake(),
  goIntake(),
  new WaitCommand(deafualtWait),
  stopIntake(),
  setPivotStored()
 );

}





// package frc.robot.subsystems.superStructure;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.subsystems.Elevator.Elevator;
// import frc.robot.subsystems.Intake.Intake;
// import frc.robot.subsystems.Pivot.Pivot;

// public class SuperStructure extends SubsystemBase{
    
//     Elevator elevator = new Elevator();
//     Pivot pivot = new Pivot();
//     Intake intake = new Intake();
//     private final int deafualtWait = 10;

//     public Command reefHighCommand = Commands.sequence(
//         elevator.elevatorUpperReef(),
//         new WaitCommand(deafualtWait), // #TODO make da waity shorter wen slo
//         pivot.reefIntake(),
//         intake.intakeIn_CMD()
//     );

//     public final Command reefLowCommand = Commands.sequence(
//         elevator.elevatorUpperReef(),
//         new WaitCommand(deafualtWait), // #TODO make da waity shorter wen slo
//         pivot.reefIntake(),
//         intake.intakeIn_CMD()
//     );
        
//     public Command groundIntakeCommand = Commands.sequence(
//         elevator.elevatorGroundIntake(),
//         new WaitCommand(deafualtWait),
//         pivot.groundIntake(),
//         intake.intakeIn_CMD()
//     );

//     public Command processorScoreCommand = Commands.sequence(
//         elevator.elevatorProcessor(),
//         new WaitCommand(deafualtWait),
//         pivot.homeScore()
//     );

//     public Command bargeScoreCommand = Commands.sequence(
//         elevator.elevatorScoreBarge(),
//         new WaitCommand(deafualtWait)
//         // pivot.bargeScore(); TODO: make dis worky
//     );

//     @Override
//     public void periodic(){
//         elevator.periodic();
//         intake.periodic();
//         pivot.periodic();
//     }
// }