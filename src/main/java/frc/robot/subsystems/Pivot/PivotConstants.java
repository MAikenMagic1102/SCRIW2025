// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Pivot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class PivotConstants {
    public static String busname = "rio";
    public static int motorID = 32;
    // public static int cancoderID = 311;

    public static double pivotGearing = 59;
    // public static double pivotGearingCANcoder = 3.5;
    // public static double pivotRotorToSensor = 25.67;
    public static double pivotLength = Units.inchesToMeters(25);
    public static double pivotMass = Units.lbsToKilograms(5.0);
    public static double pivotMinAngle = Units.degreesToRadians(-255.0);
    public static double pivotMaxAngle = Units.degreesToRadians(255.0);
    public static double pivotStartingAngle = Units.degreesToRadians(0.0);

    public static double positionTolerence = 7.0;

    public static double driveSpeed = 1.0;

    public static double lowerReef = -275; //85;
    public static double upperReef = -256; //110;
    public static double intakePos = -245;
    public static double bargeScore = -245;
    public static double stored = -280;

    public static TalonFXConfiguration config = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(70)

        )  
        .withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
          )   
        
        .withSlot0(
            new Slot0Configs()
                .withKG(0.38)
                .withKD(0)
                .withKP(60.0)
                .withKV(0.5)
        );

}