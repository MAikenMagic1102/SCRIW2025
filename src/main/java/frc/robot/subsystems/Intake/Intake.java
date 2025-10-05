package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.IntakeConstants.IntakeSpeeds;

public class Intake extends SubsystemBase{
    private TalonFX intakeRoller;
    private DigitalInput intakeSensor;
    
    public Intake(){
        intakeRoller = new TalonFX(IntakeConstants.m_intakeMotor, "rio");
        intakeSensor = new DigitalInput(IntakeConstants.intakeSensor);

        TalonFXConfiguration intakeConfiguration = new TalonFXConfiguration();
        // intakeConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakeConfiguration.CurrentLimits.SupplyCurrentLimit = 60;
        
        intakeConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        intakeRoller.getConfigurator().apply(intakeConfiguration);            
    }
    
    public double getRollerPOS(){
        return intakeRoller.getPosition().getValueAsDouble();
    }
    
    public void intakeIn(){
        if(!intakeSensor.get()){
            intakeRoller.set(IntakeConstants.IntakeSpeeds.intakeIn);
        }else{
            intakeRoller.set(IntakeConstants.IntakeSpeeds.intakeIn);
        }
    }

    public Command intakeIn_CMD(){
        return Commands.run(() ->intakeIn());
    }

    public void intakeOut(){
        intakeRoller.set(IntakeConstants.IntakeSpeeds.intakeOut);

    }

    public void rest(){
        intakeRoller.set(IntakeConstants.IntakeSpeeds.rest);

    }

    public void hold(){
        intakeRoller.set(IntakeConstants.IntakeSpeeds.hold);
    }

    @Override
    public void periodic(){

    }
    
}
