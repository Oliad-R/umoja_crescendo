package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class LowerArm extends Command{
    Arm arm;
    Intake intake;
    double armPosition;

    public LowerArm(Arm arm, Intake intake){
        this.arm = arm;
        this.intake = intake;

        addRequirements(arm, intake);
    }

    @Override
    public void execute(){
        intake.runIntake(0.6); //0.3;
        intake.runShooter(0);

        armPosition = arm.getArmPosition();

        arm.runArm(0.3*arm.armPID.calculate(armPosition, 0));
    }

    @Override
    public void end(boolean isInterrupted){
        arm.runArm(0);
    }

    @Override
    public boolean isFinished(){
        return arm.getArmLimitSwitch();
    }
}
