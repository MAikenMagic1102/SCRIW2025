package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeConstants {
    public final int intakeMoter = 0;

    public final static int m_intakeMotor = 7;
    public final static int intakeSensor = 8;

 
   public static final class IntakeSpeeds {
    public static final double intakeIn = 0.0;
    public static final double intakeOut = 0.0;
    public static final double rest = 0.0;
    public static final double hold = 0.0;
   }

   public static final class IntakePID {
    public static final double IntakekP = 0.0;
    public static final double IntakekI = 0.0;
    public static final double Intakekd = 0.0;
    public static final double IntakekS = 0.0;
    public static final double IntakekV = 0.0;
   }

   public static final double MaxIntakeSpeed = 0.0;
   public static final double MinIntakeSpeed = 0.0;

    

    

}
