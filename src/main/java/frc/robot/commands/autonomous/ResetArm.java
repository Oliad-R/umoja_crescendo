package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class ResetArm extends Command{
    Arm arm;
    Intake intake;

    public ResetArm(Arm arm, Intake intake){
        this.arm = arm;
        this.intake = intake;

        addRequirements(arm, intake);
    }

    @Override
    public void execute(){
        arm.runArm(0.5);
        intake.stop();
    }

    @Override
    public boolean isFinished(){
        return arm.getArmLimitSwitch();
    }

    @Override
    public void end(boolean isInterrupted){
        intake.stop();
        arm.resetEncoders();
    }
}
