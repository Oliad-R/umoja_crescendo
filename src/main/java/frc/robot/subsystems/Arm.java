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
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase{
    public static final CANSparkMax rightMotor = new CANSparkMax(ArmConstants.rightMotorID, CANSparkLowLevel.MotorType.kBrushless);
    public static final CANSparkMax leftMotor = new CANSparkMax(ArmConstants.leftMotorID, CANSparkLowLevel.MotorType.kBrushless);

    private static final RelativeEncoder rightEncoder = rightMotor.getEncoder(Type.kHallSensor, 42);
    private static final RelativeEncoder leftEncoder = leftMotor.getEncoder(Type.kHallSensor, 42);

    public static final DigitalInput armLimitSwitch = new DigitalInput(2);

    public static final PIDController armPID = new PIDController(ArmConstants.kP,0,0);

    private boolean isArmSetup = false;

    public Arm(){
        setArmIdleMode(IdleMode.kCoast);

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
            if(percent<0){ //If they want to move the arm up, allow it.
                leftMotor.set(percent);
                rightMotor.set(percent);
            } else { //Don't let them bring the arm back down
                leftMotor.set(0);
                rightMotor.set(0);
            }
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ARM ENCODER", getEncoderPosition());
        if (armLimitSwitch.get()) {
            resetEncoders();
        }

        if (!isArmSetup) {
            if (Math.abs(getEncoderPosition() - Constants.ArmConstants.setupPosition) < 0.5) {
                setArmIdleMode(IdleMode.kBrake);
                isArmSetup = true;
            }
        }
    }

    public double getEncoderPosition() {
        return rightEncoder.getPosition();
    }

    public void resetEncoders(){
        rightEncoder.setPosition(0);
        leftEncoder.setPosition(0);
    }

    public boolean getArmLimitSwitch(){
        return armLimitSwitch.get();
    }

    public void setArmIdleMode(IdleMode mode) {
        leftMotor.setIdleMode(mode);
        rightMotor.setIdleMode(mode);
    }
}