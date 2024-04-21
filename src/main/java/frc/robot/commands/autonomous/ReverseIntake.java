package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class ReverseIntake extends Command {
    Intake intake;

    public ReverseIntake(Intake intake){
        this.intake = intake;
        
        addRequirements(intake);
    }

    @Override
    public void execute(){
        intake.runIntake(-0.2);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}
