package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase{
    public static final CANSparkMax leftWheel = new CANSparkMax(IntakeConstants.leftWheelID, CANSparkLowLevel.MotorType.kBrushless);
    public static final CANSparkMax rightWheel = new CANSparkMax(IntakeConstants.rightWheelID, CANSparkLowLevel.MotorType.kBrushless);
    public static final CANSparkMax frontWheel = new CANSparkMax(IntakeConstants.frontWheelID, CANSparkLowLevel.MotorType.kBrushless);

    public static final DigitalInput intakeLimitSwitch = new DigitalInput(0);

    /**
     * Runs during RobotInit(). Define idlemodes, current limits, etc.
     */
    public Intake() {
        leftWheel.setIdleMode(IdleMode.kCoast);
        rightWheel.setIdleMode(IdleMode.kCoast);
        frontWheel.setIdleMode(IdleMode.kCoast);

        rightWheel.setInverted(true);
        leftWheel.setInverted(true);
        
        // leftWheel.setSmartCurrentLimit(0);
    }

    /**
     * @param percent should be positive to intake.
     */
    public void runIntake(double percent){
        frontWheel.set(percent);
    }

    /**
     * @param percent should be negative to shoot out.
     */
    public void runShooter(double percent){
        rightWheel.set(percent);
        leftWheel.set(percent);
    }

    /**
     * Stops all intake & shooting motors.
     */
    public void stop(){
        runIntake(0);
        runShooter(0);
    }

    // public boolean getIntakeLimitSwitch(){
    //     return intakeLimitSwitch.get();
    // }
}