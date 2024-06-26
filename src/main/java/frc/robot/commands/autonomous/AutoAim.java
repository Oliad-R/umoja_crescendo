package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class AutoAim extends Command{
    Arm arm;
    Intake intake; 
    double armPosition, goal;

    public AutoAim(Arm arm, Intake intake){
        goal = ArmConstants.speakerEncoder;
        this.arm = arm;
        this.intake = intake;

        addRequirements(arm, intake);
    }

    @Override
    public void initialize() {
        intake.isShooting = true;
        intake.runShooter(-0.7);
        intake.runIntake(0);
    }

    @Override
    public void execute(){
        armPosition = arm.getArmPosition();
        arm.runArm(arm.armPID.calculate(armPosition, goal));
    }

    @Override
    public boolean isFinished(){
        intake.isShooting = false;
        return !(Math.abs(armPosition-goal) > 0.6);
    }

    @Override
    public void end(boolean isInterrupted){
        arm.runArm(0);
    }
}
