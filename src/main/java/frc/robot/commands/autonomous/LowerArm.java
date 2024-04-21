package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
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
    public void initialize() {
        intake.runShooter(0);
        intake.runIntake(0.6);
        if (intake.hasNote) {
            intake.runIntake(0.2);
        }
    }

    @Override
    public void execute(){
        armPosition = arm.getArmPosition();
        arm.runArm(0.5*arm.armPID.calculate(armPosition, ArmConstants.armHover));

        if (arm.getArmLimitSwitch()) {
            arm.runArm(0);
        }
    }

    @Override
    public void end(boolean isInterrupted){
        arm.runArm(0);
        intake.runShooter(0.2);
    }

    @Override
    public boolean isFinished(){
        return arm.getArmLimitSwitch();
    }
}
