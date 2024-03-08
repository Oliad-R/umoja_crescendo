package frc.robot.commands;

import java.util.function.Supplier;

import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.USB;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;

public class ArmJoystick extends Command {
    //Defining subsystems
    Arm armSubsystem;
    Intake intakeSubsystem;
    Climber climberSubsystem;

    Joystick j = new Joystick(USB.OPERATOR_CONTROLLER);
    double armInput;

    public ArmJoystick(Arm armSubsystem, Intake intakeSubsystem, Climber climberSubsystem, Joystick j){
        this.armSubsystem = armSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.climberSubsystem = climberSubsystem;

        //Add subsystem dependencies
        addRequirements(armSubsystem, intakeSubsystem, climberSubsystem);
    }

    @Override
    public void execute(){
        boolean isShooting = j.getRawButton(OIConstants.LB);
        if(isShooting){
            intakeSubsystem.runShooter(-1);
        } else {
            intakeSubsystem.runShooter(0);
        }
        if(j.getRawButton(OIConstants.RB)){
            if (!isShooting && intakeSubsystem.intakeLimitSwitch.get()) {
                intakeSubsystem.runIntake(0);
            } else {
                intakeSubsystem.runIntake(1);
            }
        } else {
            intakeSubsystem.runIntake(0);
        }

        if(j.getRawButton(OIConstants.A)){
            climberSubsystem.runClimber(-0.3);
        } else if(j.getRawButton(OIConstants.Y)){
            climberSubsystem.runClimber(0.3);
        } else {
            climberSubsystem.runClimber(0);
        }
        // else if(j.getRawButton(5) && j.getRawButton(6)){
        //     intakeSubsystem.runShooter(-1);
        //     intakeSubsystem.runIntake(1);
        // }
        // if (j.getRawButton(1)){
        //     climberSubsystem.CLIMB_STATE=1;
        // }
        // else if (j.getRawButton(4)){
        //     climberSubsystem.CLIMB_STATE=0;
        // }
        // else {
        //     intakeSubsystem.stop();
        //     climberSubsystem.stop();
        // }

        armInput = j.getRawAxis(OIConstants.LY)*0.4;

        SmartDashboard.putNumber("JVALUE", armInput);

        if(!armSubsystem.armLimitSwitch.get()){
            armSubsystem.runArm(armInput);
        } else {
            armSubsystem.rightEncoder.setPosition(0);
            armSubsystem.leftEncoder.setPosition(0);
            if(armInput<0){
                armSubsystem.runArm(armInput);
            } else {
                armSubsystem.runArm(0);
            }
        }

        SmartDashboard.putBoolean("LIMITSWITCH ARM", armSubsystem.armLimitSwitch.get());
        
        SmartDashboard.putNumber("CLIMBER ENCODER", climberSubsystem.rightMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("ARM ENCODER", armSubsystem.rightEncoder.getPosition());

        double position = armSubsystem.rightEncoder.getPosition();


        VisionLEDMode ledMode = position < -141 || ( position < -22 && position > -25) ? VisionLEDMode.kOn : VisionLEDMode.kOff;
        RobotContainer.camera.setLED(ledMode);
        // armSubsystem.runArm()
    }
}