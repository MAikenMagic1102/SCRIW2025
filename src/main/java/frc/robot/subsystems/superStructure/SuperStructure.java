package frc.robot.subsystems.superStructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Pivot;

public class SuperStructure extends Command{
    
    Elevator elevator = new Elevator();
    Pivot pivot = new Pivot();
    Intake intake = new Intake();
    private final int deafualtWait = 10;

    public Command reefHighCommand = Commands.sequence(
        elevator.elevatorUpperReef(),
        new WaitCommand(deafualtWait), // #TODO make da waity shorter wen slo
        pivot.reefIntake(),
        intake.intakeIn_CMD()
    );

    public final Command reefLowCommand = Commands.sequence(
            elevator.elevatorUpperReef(),
            new WaitCommand(deafualtWait), // #TODO make da waity shorter wen slo
            pivot.reefIntake(),
            intake.intakeIn_CMD()
    );
        
    public Command groundIntakeCommand = Commands.sequence(
        elevator.elevatorGroundIntake(),
        new WaitCommand(deafualtWait),
        pivot.groundIntake(),
        intake.intakeIn_CMD()
    );

    public Command processorScoreCommand = Commands.sequence(
        elevator.elevatorProcessor(),
        new WaitCommand(deafualtWait),
        pivot.homeScore()
    );

    public Command bargeScoreCommand = Commands.sequence(
        elevator.elevatorScoreBarge(),
        new WaitCommand(deafualtWait)
        // pivot.bargeScore(); TODO: make dis worky
    );
}