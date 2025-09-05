package frc.robot.subsystems.superStructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Pivot;

public class SuperStructure extends SubsystemBase{
    
    Elevator elevator;
    Pivot pivot;
    Intake intake;
    private final int deafualtWait = 10;

    public static Command reefHighCommand; {
        elevator.elevatorUpperReef();
        new WaitCommand(deafualtWait); // #TODO make da waity shorter wen slo
        pivot.reefIntake();
        intake.intakeIn();
    };

    public static Command reefLowCommand; {
        elevator.elevatorLowerReef();
        new WaitCommand(deafualtWait);
        pivot.reefIntake();
        intake.intakeIn();
    };
        
    public static Command groundIntakeCommand; {
        elevator.elevatorGroundIntake();
        new WaitCommand(deafualtWait);
        pivot.groundIntake();
        intake.intakeIn();
    };

    public static Command processorScoreCommand; {
        elevator.elevatorProcessor();
        new WaitCommand(deafualtWait);
        pivot.homeScore();
    };

    public static Command bargeScoreCommand; {
        elevator.elevatorScoreBarge();
        new WaitCommand(deafualtWait);
        // pivot.bargeScore(); TODO: make dis worky
    };
}