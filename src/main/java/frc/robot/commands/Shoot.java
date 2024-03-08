package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.USB;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;

public class Shoot extends Command {
    Arm armSubsystem;
    Intake intakeSubsystem;
    Climber climberSubsystem;

    public Shoot(Arm armSubsystem, Intake intakeSubsystem, Climber climberSubsystem){
        this.armSubsystem = armSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.climberSubsystem = climberSubsystem;

        //Add subsystem dependencies
        addRequirements(armSubsystem, intakeSubsystem, climberSubsystem);
    }

    @Override
    public void execute(){
        if(armSubsystem.rightEncoder.getPosition() > -14){
            armSubsystem.runArm(-0.5);
        } else {
            armSubsystem.runArm(0);
            intakeSubsystem.runShooter(1);
        }
    }
}
