package frc.robot.subsystems.superStructure;

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

    public void reefHigh() {
        elevator.elevatorUpperReef();
        new WaitCommand(deafualtWait); // #TODO make da waity shorter wen slo
       // pivot.reefIntake();
        intake.intakeIn();
    };

    public void reefLow(){
        elevator.elevatorLowerReef();
        new WaitCommand(deafualtWait);
        //pivot.reefIntake();
        intake.intakeIn();
    };
        
    public void groundIntake() {
        elevator.elevatorGroundIntake();
        new WaitCommand(deafualtWait);
        //pivot.groundIntake();
        intake.intakeIn();
    };

    public void processorScore(){
        elevator.elevatorProcessor();
        new WaitCommand(deafualtWait);
        //pivot.homeScore();
    };

    public void bargeScore(){
        elevator.elevatorScoreBarge();
        new WaitCommand(deafualtWait);
        // pivot.bargeScore(); TODO: make dis worky
    };
}