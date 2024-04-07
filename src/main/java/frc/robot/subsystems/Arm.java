package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkRelativeEncoder.Type;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase{
    private CANSparkMax rightMotor = new CANSparkMax(ArmConstants.rightMotorID, CANSparkLowLevel.MotorType.kBrushless);
    private CANSparkMax leftMotor = new CANSparkMax(ArmConstants.leftMotorID, CANSparkLowLevel.MotorType.kBrushless);

    private RelativeEncoder rightEncoder = rightMotor.getEncoder(Type.kHallSensor, 42);
    private RelativeEncoder leftEncoder = leftMotor.getEncoder(Type.kHallSensor, 42);

    private boolean isArmReady = false;

    public static final DigitalInput armLimitSwitch = new DigitalInput(2);

    public static final PIDController armPID = new PIDController(ArmConstants.kP,0,0);

    public Arm(){
        setIdleMode(IdleMode.kCoast);

        leftMotor.setSmartCurrentLimit(30);
        rightMotor.setSmartCurrentLimit(30);

        // rightMotor.follow(leftMotor, true);
        rightMotor.setInverted(true);
    }

    /**
     * @param percent should be negative/positive to move the arm up/down.
     */
    public void runArm(double percent){
        if(!armLimitSwitch.get()){ //If the arm isn't fully down operate regularly
            leftMotor.set(percent);
            rightMotor.set(percent);
        } else { //If the arm is down:
            resetEncoders();
            if(percent<0){ //If they want to move the arm up, allow it.
                leftMotor.set(percent);
                rightMotor.set(percent);
            } else { //Don't let them bring the arm back down
                leftMotor.set(0);
                rightMotor.set(0);
            }
        }
    }

    public void setIdleMode(IdleMode mode){
        leftMotor.setIdleMode(mode);
        rightMotor.setIdleMode(mode);
    }

    public void resetEncoders(){
        rightEncoder.setPosition(0);
        leftEncoder.setPosition(0);
    }

    public boolean getArmLimitSwitch(){
        return armLimitSwitch.get();
    }

    /**
     * Uses the right encoder.
     * @return armPosition (double)
     */
    public double getArmPosition(){
        return rightEncoder.getPosition();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("ARM ENCODER", getArmPosition());

        if(getArmLimitSwitch()){
            resetEncoders();
        }

        if(!isArmReady && getArmPosition() < ArmConstants.armStartingPos){
            setIdleMode(IdleMode.kBrake);
            isArmReady = true;
        }

    }
}