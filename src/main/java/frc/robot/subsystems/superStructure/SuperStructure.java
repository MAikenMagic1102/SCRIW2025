// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superStructure;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;
// import frc.robot.subsystems.Mechanism.SuperStructureMechanism;

public class SuperStructure extends SubsystemBase {

  private final Elevator elevator = new Elevator ();

  // private final SuperStructureMechanism mech = new SuperStructureMechanism();

  private enum scoreTarget{
    Lower,
    Upper,
    ALGAE
  };

  private scoreTarget currentTarget = scoreTarget.Lower;

  private double armTargetAngle = 0.0;
  private double elevatorTargetHeight = 0.0;

  private boolean algaeNext = false;
  
  /** Creates a new Superstructure. */
  public SuperStructure() {}

  @Override
  public void periodic() {
    SmartDashboard.putString("Score Target", currentTarget.toString());
    
    // This method will be called once per scheduler run
    elevator.periodic();
    // mech.update(elevator.getPositionMeters());
    elevatorTargetHeight = getScoreTargetElevatorPos();

    Logger.recordOutput("Superstructure/ Score Target", currentTarget.toString());
    Logger.recordOutput("Superstructure/ Elevator Target Height", elevatorTargetHeight);
    Logger.recordOutput("Superstructure/ Arm Target Angle", armTargetAngle);
    Logger.recordOutput("Superstructure/ Algae Next", algaeNext);

    SmartDashboard.putNumber("Elevator Target Height", elevatorTargetHeight);
    SmartDashboard.putNumber("Arm Target Angle", armTargetAngle);
    SmartDashboard.putBoolean("Algae NEXT", algaeNext);
  }

  public Command setAlgaeNext(){
    return new InstantCommand(() -> algaeNext = true);
  }

  public Command setAlgaeNextOFF(){
    return new InstantCommand(() -> algaeNext = false);
  }

  public boolean getAgaeState(){
    return algaeNext;
  }

  public boolean elevatorAtGoal(){
    return elevator.atGoal();
  }

  public Command setTargetL1(){
    return runOnce(() -> currentTarget = scoreTarget.Lower);
  }

  public Command setTargetL2(){
    return runOnce(() -> currentTarget = scoreTarget.Upper);
  }

  public Command setTargetAlgae(){
    return runOnce(() -> currentTarget = scoreTarget.ALGAE);
  }


  public double getScoreTargetElevatorPos(){
    double scoreTarget = 0.0;

    switch(currentTarget){
      case Lower:
        scoreTarget = ElevatorConstants.reefLower;
      break;
      case Upper:
        scoreTarget = ElevatorConstants.reefUpper;
      break;
      case ALGAE:
      scoreTarget = ElevatorConstants.ALGAE;
      break;                  
    }
    
    return scoreTarget;
  }

  public Command setElevatorToScore(){
    return new InstantCommand(() -> elevator.setPositionMetersMM(elevatorTargetHeight));
  }

  public boolean isElevatorAtGoal(){
    return elevator.atGoal();
  }

  public boolean isElevatorSafe(){
    return elevator.belowHalf();
  }

  public boolean isElevatorBelowHalf(){
    return elevator.belowHalf();
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
    return new InstantCommand(() -> elevator.setOpenLoop(0));
  }

  public Command setMiddlePos() {
     return new InstantCommand(() -> elevator.setPositionMetersMM(0.85));
 }

 public Command setHighPos(){
  return new InstantCommand(() -> elevator.setPositionMeters(1));
 }  

 public Command setElevatorHome(){
  return new InstantCommand(() -> elevator.setPositionMetersMM(0.007));
 }  

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