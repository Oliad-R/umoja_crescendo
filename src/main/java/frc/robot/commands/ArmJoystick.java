package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.USB;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;

public class ArmJoystick extends Command {
    //Defining subsystems
    Arm armSubsystem;
    Intake intakeSubsystem;
    Climber climberSubsystem;

    Joystick j = new Joystick(USB.OPERATOR_CONTROLLER);
    double armInput, armPos, error;
    PIDController armPID = new PIDController(ArmConstants.kP, 0, ArmConstants.kD);
    boolean useFlatAngle, defaultArmDown = true;

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
            intakeSubsystem.runShooter(-0.7);
        } else {
            intakeSubsystem.runShooter(0);
        }

        if(j.getRawButton(OIConstants.RB)){
            intakeSubsystem.runIntake(1);
        } else {
            intakeSubsystem.runIntake(0);
        }

        //Climber Logic
        if(j.getRawAxis(OIConstants.RT) > 0.5){
            climberSubsystem.runClimber(-0.3);
        } else if(j.getRawAxis(OIConstants.LT) > 0.5) {
            climberSubsystem.runClimber(0.3);
        } else {
            climberSubsystem.runClimber(0);
        }

        //Running the arm
        armInput = j.getRawAxis(OIConstants.LY)*0.4;

        if (j.getRawButton(OIConstants.B)) {
            armPos = armSubsystem.getArmPosition();
            armSubsystem.runArm(armPID.calculate(armPos, ArmConstants.speakerEncoder));
        } else if (j.getRawButton(OIConstants.Y)) {
            armPos = armSubsystem.getArmPosition();
            armSubsystem.runArm(armPID.calculate(armPos, ArmConstants.farSpeakerEncoder));
        } else if (j.getRawButton(OIConstants.A)) {
            armPos = armSubsystem.getArmPosition();
            armSubsystem.runArm(armPID.calculate(armPos, ArmConstants.ampEncoder));
        } else {
            if (defaultArmDown) {
                armPos = armSubsystem.getArmPosition();
                armSubsystem.runArm(0.5*armPID.calculate(armPos, ArmConstants.armHover));
            } else {
                armSubsystem.runArm(armInput);
            }
        }

        if(j.getRawButtonPressed(OIConstants.START)){
            defaultArmDown = !defaultArmDown;
        }



        //Reverse the intake
        if(j.getRawButton(OIConstants.X)){
            intakeSubsystem.runIntake(-0.25);
        }

        // double armPos = armSubsystem.getArmPosition();

        // VisionLEDMode currentLedMode = RobotContainer.camera.getLEDMode();
        // VisionLEDMode ledMode = armPos < -141 || ( armPos < ArmConstants.speakerEncoder && armPos > -24.5) ? VisionLEDMode.kOn : VisionLEDMode.kOff;

        // if(ledMode!=currentLedMode){ //Should reduce how many times we set the camera LED mode
        //     RobotContainer.camera.setLED(ledMode);
        // }
        // armSubsystem.runArm()
    }
}