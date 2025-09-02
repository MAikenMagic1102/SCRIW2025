package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.Command;

public class Intake {
    private static TalonFX intakeRoller;
    private static DigitalInput intakeSensor;
        
    
        public Intake(){
            intakeRoller = new TalonFX(IntakeConstants.m_intakeMotor);
            intakeSensor = new  DigitalInput(IntakeConstants.intakeSensor);
            TalonFXConfiguration intakeConfiguration = new TalonFXConfiguration();
            
        }
    
        double motorPosRotations = intakeRoller.getPosition().getValueAsDouble();
    
    
        public static void intakeIn(){
            if(!intakeSensor.get()){
            intakeRoller.set(IntakeConstants.IntakeSpeeds.intakeIn);
        }else{
            intakeRoller.set(IntakeConstants.IntakeSpeeds.intakeIn);
        }

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
    
}
