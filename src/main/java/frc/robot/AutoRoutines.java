package frc.robot;

import java.util.function.Supplier;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;



public class AutoRoutines {

    private final AutoFactory m_factory;
    private final CommandSwerveDrivetrain drive;

    public AutoRoutines(CommandSwerveDrivetrain drive) {

        this.drive = drive;
        m_factory = drive.createAutoFactory();
    }
public AutoRoutine Red_Mid_Frwd(){
    AutoRoutine routine = m_factory.newRoutine("Red_Mid_Frwd");
    final AutoTrajectory redMiddleForeward = routine.trajectory("RedMiddleForeward");

    routine.active().onTrue(
    redMiddleForeward.resetOdometry().andThen(redMiddleForeward.cmd()));
    return routine;
}

public AutoRoutine Red_Lft_Frwd(){
    AutoRoutine routine = m_factory.newRoutine("Red_Lft_Frwd");
    final AutoTrajectory redLeftForeward = routine.trajectory("RedLeftForeward");

    routine.active().onTrue(
    redLeftForeward.resetOdometry().andThen(redLeftForeward.cmd()));
    return routine;
}

public AutoRoutine Red_Rgt_Frwd(){
    AutoRoutine routine = m_factory.newRoutine("Red_Rgt_Frwd");
    final AutoTrajectory redRightForeward = routine.trajectory("RedRightForeward");

    routine.active().onTrue(
    redRightForeward.resetOdometry().andThen(redRightForeward.cmd()));
    return routine;
}

public AutoRoutine Blue_Mid_Frwd(){
    AutoRoutine routine = m_factory.newRoutine("Blue_Mid_Frwd");
    final AutoTrajectory blueMiddleForeward = routine.trajectory("BlueMiddleForeward");

    routine.active().onTrue(
    blueMiddleForeward.resetOdometry().andThen(blueMiddleForeward.cmd()));
    return routine;
}


public AutoRoutine Blue_Lft_Frwd(){
    AutoRoutine routine = m_factory.newRoutine("Blue_Lft_Frwd");
    final AutoTrajectory blueLeftForeward = routine.trajectory("BlueLeftForeward");

    routine.active().onTrue(
    blueLeftForeward.resetOdometry().andThen(blueLeftForeward.cmd()));
    return routine;
}

public AutoRoutine Blue_Rgt_Frwd(){
    AutoRoutine routine = m_factory.newRoutine("Blue_Rgt_Frwd");
    final AutoTrajectory blueRightForeward = routine.trajectory("BlueRightForeward");

    routine.active().onTrue(
    blueRightForeward.resetOdometry().andThen(blueRightForeward.cmd()));
    return routine;
}
 


    // public AutoRoutine Red_Mid_Frwd(){
    //     final AutoRoutine = m_factory.newRoutine("RedMiddleForeward");

    // }    
    
}
