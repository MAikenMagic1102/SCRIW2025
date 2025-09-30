package frc.robot.subsystems;
import static edu.wpi.first.units.Units.*;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
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
import frc.robot.subsystems.Elevator.ElevatorConstants;


public class Climber {
    //TODO: Write climber logic. I could use some help, since I have no Idea what I'm doing. Thanks, Codecademy
    //Maybe I can look through the other code as reference...
final TalonFX motorClimber = new TalonFX(0, "rio");
final DutyCycleOut motorClimberRequest = new DutyCycleOut(0.0);
StatusSignal<Angle> rotorPosSignal = motorClimber.getRotorPosition();
Angle rotorPos = rotorPosSignal.getValue();
public double currentAngle = rotorPosSignal.getValueAsDouble();
//I don't know what I'm doing but I'm doing something :3
public void neutralPos() {
    while (currentAngle >= 211 && currentAngle <= 209) {
        motorClimber.setControl(new Follower(0, true));

        motorClimber.setControl(motorClimberRequest.withOutput(1));
    }

}
public void extendedPos() {

}
}
