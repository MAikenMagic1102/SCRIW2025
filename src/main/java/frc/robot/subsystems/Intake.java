package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.Command;

public class Intake {
    private TalonFX intakeRoller;
    private DigitalInput intakeSensor;
    

    public Intake(){
        intakeRoller = new TalonFX(0);
        intakeSensor = new  DigitalInput(0);
        TalonFXConfiguration intakeConfiguration = new TalonFXConfiguration();
        
    }

    double motorPosRotations = intakeRoller.getPosition().getValueAsDouble();


    public void intakeIn(){
        if(!intakeSensor.get()){
            intakeRoller.set(0);
        }else{
            intakeRoller.set(0);
        }

    }
    public void intakeOut(){
        intakeRoller.set(0);

    }
    public void rest(){
        intakeRoller.set(0);

    }
    public void hold(){
        intakeRoller.set(0);
    }
    
}
