package frc.robot.subsystems.Elevator;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;




public class Elevator extends SubsystemBase{

    // Define TalonFX motor objects
    private TalonFX m_motorLeft;
    private TalonFX m_motorRight;

    private double cmPerRot = ElevatorConstants.cmPerRot;
    private double cmElevatorZero = ElevatorConstants.cmElevatorZero;

    // Get motor rotations
    private StatusSignal<Angle> motorLeftRot = m_motorLeft.getPosition();
    private StatusSignal<Angle> motorRightRot = m_motorRight.getPosition();

    final DutyCycleOut m_dutyLeft = new DutyCycleOut(0.0);
    final DutyCycleOut m_dutyRight = new DutyCycleOut(0.0);

    
    private double positionError;
   
    private double acceptableError = 0.05;

    private void sendElevatorToPoint(double point){
        positionError = getError(point);
        double acceptableErrorLower = (1 - acceptableError) * point;
        double acceptableErrorUpper = (1 + acceptableError) * point;
        boolean withinError = acceptableErrorLower < positionError && positionError < acceptableErrorUpper;
        double motorPercent = 0.1;

        // Negative error means that our elevator is higher than the desired position
        if(positionError < 0){
            // Negative percent moves elevator down
            motorPercent *= -1;
        }

        // Keep applying power until elevator is within 5% of the desired location
        // TODO: WARNING: UNTESTED CODE! NOT FOR USE ON ROBOT! FIGURE OUT SIM FIRST! FIGURE OUT WHICH WAY THE MOTORS ROTATE
        while(!withinError){
            m_motorLeft.setControl(m_dutyLeft.withOutput(motorPercent));
            m_motorRight.setControl(m_dutyRight.withOutput(motorPercent));
        };
    };

    public void elevatorGroundIntake(){
       sendElevatorToPoint(ElevatorConstants.ElevatorSetpoints.elevatorGroundIntake);
    };

    public void elevatorLowerReef() {};

    public void elevatorUpperReef() {};

    public void elevatorHome() {};

    public void elevatorScoreBarge() {};
    
    private double getElevatorHeight(){
        // TODO: FLESH OUT! FIGURE OUT LEAD MOTOR!
        double elevatorHeight = cmPerRot * motorLeftRot.getValueAsDouble() + cmElevatorZero;
        return elevatorHeight;
        // Will return the result of our equation defining elevator height based on rotations.
        //TODO: Get elevator height equation. Zero is ABOUT 36 inches. Need to confirm.
    };

    private double getError(double setpoint){
        // Return error between current height and desired position
        return setpoint - getElevatorHeight();
    };

}
