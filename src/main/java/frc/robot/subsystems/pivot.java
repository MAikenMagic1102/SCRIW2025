package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

public class Pivot {
public double homeScoreAngle = 0;
public double groundIntakeAngle = 0;
public double reefIntakeAngle = 0;
final TalonFX motorPivot = new TalonFX(7, "rio");
final DutyCycleOut motorPivotRequest = new DutyCycleOut(0.0);

StatusSignal<Angle> rotorPosSignal = motorPivot.getRotorPosition();
Angle rotorPos = rotorPosSignal.getValue();

public double currentAngle = rotorPosSignal.getValueAsDouble();

    public Command groundIntake; {
        while (currentAngle >= 211 && currentAngle <= 209) {
            motorPivot.setControl(motorPivotRequest.withOutput(1));
        }
    }
    
    public Command reefIntake; {
        while (currentAngle >= 121 && currentAngle <= 119) {
            motorPivot.setControl(motorPivotRequest.withOutput(1));
        }
    }

    public Command homeScore; {
        while (currentAngle >= 1 && currentAngle <= -1) {
            motorPivot.setControl(motorPivotRequest.withOutput(1));
        }
    }
   
};
