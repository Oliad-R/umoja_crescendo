package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkRelativeEncoder.Type;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase{
    public static final CANSparkMax rightMotor = new CANSparkMax(ArmConstants.rightMotorID, CANSparkLowLevel.MotorType.kBrushless);
    public static final CANSparkMax leftMotor = new CANSparkMax(ArmConstants.leftMotorID, CANSparkLowLevel.MotorType.kBrushless);

    public static final RelativeEncoder rightEncoder = rightMotor.getEncoder(Type.kHallSensor, 42);
    public static final RelativeEncoder leftEncoder = leftMotor.getEncoder(Type.kHallSensor, 42);

    public static final DigitalInput armLimitSwitch = new DigitalInput(2);

    public static final PIDController armPID = new PIDController(0,0,0);

    public Arm(){
        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);

        leftMotor.setSmartCurrentLimit(30);
        rightMotor.setSmartCurrentLimit(30);

        // rightMotor.follow(leftMotor, true);
        rightMotor.setInverted(true);
    }

    //TO-DO:
    // put the arm limit switch in here
    public void runArm(double percent){
        leftMotor.set(percent);
        rightMotor.set(percent);
    }
}