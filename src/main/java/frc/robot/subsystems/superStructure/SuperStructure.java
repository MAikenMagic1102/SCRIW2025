package frc.robot.subsystems.superStructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Intake;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SuperStructure extends SubsystemBase{
    
    Elevator elevator;
    Pivot pivot;
    Intake intake;

    public void reefHigh() {
        elevator.elevatorUpperReef();
        new WaitCommand(10); // #TODO make da waity shorter wen slo
        pivot.reefIntake();
        Intake.intakeIn();
    };

    public void reefLow(){
        
    };
        
    public void groundIntake() {

    };

    public void processorScore(){

    };

    public void bargeScore(){

    };
}