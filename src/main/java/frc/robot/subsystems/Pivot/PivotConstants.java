// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Pivot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class PivotConstants {
    public static String busname = "rio";
    public static int motorID = 40;
    public static int cancoderID = 311;

    public static double pivotGearing = 134;
    public static double pivotGearingCANcoder = 3.5;
    public static double pivotRotorToSensor = 25.67;
    public static double pivotLength = Units.inchesToMeters(25);
    public static double pivotMass = Units.lbsToKilograms(5.0);
    public static double pivotMinAngle = Units.degreesToRadians(-255.0);
    public static double pivotMaxAngle = Units.degreesToRadians(255.0);
    public static double pivotStartingAngle = Units.degreesToRadians(0.0);

    public static double positionTolerence = 7.0;

    public static double driveSpeed = 1.0;

    public static double intake = -100;
    public static double lowerReef = -275; //85;
    public static double upperReef = -256; //110;
    public static double ALGAE = -245;

    public static TalonFXConfiguration config = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(70)

        )  
        .withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
          )   
          //10:70 - 18:66 S - 10:35
        // .withFeedback(
        //     new FeedbackConfigs()
        //         .withSensorToMechanismRatio(3.5)
        //         // .withFeedbackRemoteSensorID(cancoderID)
        //         .withRotorToSensorRatio(25.67)
        //         .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
        // )
        .withSlot0( // PID tuning
            new Slot0Configs()
                .withKG(0.38)
                .withKD(0)
                //90 dg, str, fll rst bpos
                .withKP(60.0)
                .withKV(0.5)
        );

    // public static CANcoderConfiguration ccconfig = new CANcoderConfiguration()
    //     .withMagnetSensor(
    //         new MagnetSensorConfigs()
    //             .withAbsoluteSensorDiscontinuityPoint(0.0)
    //             .withMagnetOffset(-0.87)
    //             //.withMagnetOffset(1.32)
    //             .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
        // );

}