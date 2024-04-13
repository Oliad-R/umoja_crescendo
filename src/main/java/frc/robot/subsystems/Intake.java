package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.GameConstants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase{
    public static final CANSparkMax leftWheel = new CANSparkMax(IntakeConstants.leftWheelID, CANSparkLowLevel.MotorType.kBrushless);
    public static final CANSparkMax rightWheel = new CANSparkMax(IntakeConstants.rightWheelID, CANSparkLowLevel.MotorType.kBrushless);
    public static final CANSparkMax frontWheel = new CANSparkMax(IntakeConstants.frontWheelID, CANSparkLowLevel.MotorType.kBrushless);

    public static final DigitalInput intakeSensor = new DigitalInput(9);

    public boolean hasNote = false;
    public boolean intakeState;
    public int red, green, blue = 0;

    /**
     * Runs during RobotInit(). Define idlemodes, current limits, etc.
     */
    public Intake() {
        leftWheel.setIdleMode(IdleMode.kCoast);
        rightWheel.setIdleMode(IdleMode.kCoast);
        frontWheel.setIdleMode(IdleMode.kCoast);

        rightWheel.setInverted(true);
        leftWheel.setInverted(true);
        
        // leftWheel.setSmartCurrentLimit(30);
        // rightWheel.setSmartCurrentLimit(30);
        // frontWheel.setSmartCurrentLimit(30);
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

    public boolean getIntakeSwitch(){
        return !intakeSensor.get();
    }

    @Override
    public void periodic() {
        super.periodic();

        if(RobotContainer.gameState==GameConstants.TeleOp){
            intakeState = getIntakeSwitch();

            SmartDashboard.putBoolean("INTAKE SWITCH", intakeState);

            if(hasNote!=intakeState){

                if(intakeState){
                    red = 0;
                    green = 255;
                } else {
                    red = 255;
                    green = 0;
                }
                
                RobotContainer.led.setLEDColor(red, green, blue);
                hasNote = !hasNote;
            }
        }
    }
}