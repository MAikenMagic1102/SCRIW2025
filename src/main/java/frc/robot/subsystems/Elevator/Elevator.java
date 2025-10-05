package frc.robot.subsystems.Elevator;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator extends SubsystemBase{

    // Define TalonFX motor objects

    public TalonFX m_motorLeft;
    public TalonFX m_motorRight;

    private double cmPerRot = ElevatorConstants.cmPerRot;
    private double cmElevatorZero = ElevatorConstants.cmElevatorZero;

    // Get motor rotations
    private StatusSignal<Angle> motorLeftRotStatSig;
    private StatusSignal<Angle> motorRightRotStatSig;

    private double motorLeftRot;
    private double motorRightRot;

    final DutyCycleOut m_dutyLeft = new DutyCycleOut(0.0);
    final DutyCycleOut m_dutyRight = new DutyCycleOut(0.0);

    final PositionVoltage positionVoltage = new PositionVoltage(0);
    
    private double positionError;
    private double acceptableError = ElevatorConstants.errorRange;
    
    public Elevator(){
        m_motorLeft = new TalonFX(ElevatorConstants.m_motorLeft_ID);
        m_motorRight = new TalonFX(ElevatorConstants.m_motorRight_ID);

        TalonFXConfiguration motorLeft_Configuration = new TalonFXConfiguration();

        //TODO: all var & definitions

        motorLeft_Configuration.Slot0.kP = 0;

        m_motorLeft.getConfigurator().apply(motorLeft_Configuration);
        m_motorRight.setControl(new Follower(ElevatorConstants.m_motorLeft_ID, false));
    }


    private void sendElevatorToPoint(double point){
        // positionError = getError(point);
        // double acceptableErrorLower = (1 - acceptableError) * point;
        // double acceptableErrorUpper = (1 + acceptableError) * point;
        // boolean withinError = acceptableErrorLower < positionError && positionError < acceptableErrorUpper;
        // double motorPercent = 0.1;

        // //Negative error means that our elevator is higher than the desired position
        //  if(positionError < 0){
        //      // Negative percent moves elevator down
        //      motorPercent *= -1;
        //  }

        // Keep applying power until elevator is within 5% of the desired location
        // TODO: WARNING: UNTESTED CODE! NOT FOR USE ON ROBOT! FIGURE OUT SIM FIRST! FIGURE OUT WHICH WAY THE MOTORS ROTATE
        // while(!withinError){
        //     m_motorLeft.setControl(m_dutyLeft.withOutput(motorPercent));
        //     m_motorRight.setControl(m_dutyRight.withOutput(motorPercent));
            
        //     positionError = getError(point);
        //     withinError = acceptableErrorLower < positionError && positionError < acceptableErrorUpper;
        // }
    }

public void setPosition(double position){

m_motorLeft.setControl(positionVoltage.withPosition(position));
    
}

    public Command elevatorGroundIntake(){
       return Commands.runOnce(()-> sendElevatorToPoint(ElevatorConstants.ElevatorSetpoints.elevatorGroundIntake));

    };

    public Command elevatorLowerReef() {
        return Commands.runOnce(()-> sendElevatorToPoint(ElevatorConstants.ElevatorSetpoints.elevatorLowerReef));
    };

    public Command elevatorUpperReef() {
        return Commands.runOnce(()-> sendElevatorToPoint(ElevatorConstants.ElevatorSetpoints.elevatorUpperReef));
        
        // sendElevatorToPoint(ElevatorConstants.ElevatorSetpoints.elevatorLowerReef);
    };

    public Command elevatorHome() {
        return Commands.runOnce(()-> sendElevatorToPoint(ElevatorConstants.ElevatorSetpoints.elevatorHome));

        // sendElevatorToPoint(ElevatorConstants.ElevatorSetpoints.elevatorHome);
    };

    public Command elevatorScoreBarge() {
        return Commands.runOnce(()-> sendElevatorToPoint(ElevatorConstants.ElevatorSetpoints.barge));
    };

    public void elevatorTester() {

    };

    public Command elevatorProcessor() {
        return Commands.runOnce(()-> sendElevatorToPoint(ElevatorConstants.ElevatorSetpoints.processor));
    };

    public double getRotLeft() {
        motorLeftRot = m_motorLeft.getPosition().getValueAsDouble();
        return motorLeftRot;
    };

    public double getRotRight() {
        motorRightRot = m_motorRight.getPosition().getValueAsDouble();
        return motorRightRot;
    };
    
    private double getElevatorHeight(){
        // TODO: FLESH OUT! FIGURE OUT LEAD MOTOR!
        double elevatorHeight = cmPerRot * motorLeftRot + cmElevatorZero;
        return elevatorHeight;
        // Will return the result of our equation defining elevator height based on rotations.
        //TODO: Get elevator height equation. Zero is ABOUT 36 inches. Need to confirm.
    };


    private double getError(double setpoint){
        // Return error between current height and desired position
        return setpoint - getElevatorHeight();
    };

    public void rotationOnePercent() {
        m_motorLeft.setControl(m_dutyLeft.withOutput(0.01));
        m_motorRight.setControl(m_dutyRight.withOutput(0.01));
    };

    @Override
    public void periodic(){

        motorLeftRotStatSig = m_motorLeft.getPosition();
        motorRightRotStatSig = m_motorRight.getPosition();

        motorLeftRot = motorLeftRotStatSig.getValueAsDouble();
     motorRightRot = motorRightRotStatSig.getValueAsDouble();

        SmartDashboard.setDefaultNumber("lefty", getRotLeft());
        SmartDashboard.setDefaultNumber("righty", getRotRight());

        SmartDashboard.putNumber("Left Eeelvator rotations", getRotLeft()); 
        SmartDashboard.putNumber("Right Eelevator rotations", getRotRight()); 
        
    };
}
