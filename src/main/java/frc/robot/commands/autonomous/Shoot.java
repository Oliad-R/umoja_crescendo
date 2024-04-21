package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class Shoot extends Command{
    Arm arm;
    Intake intake;
    double currentTime, armPosition;

    public Shoot(Arm arm, Intake intake){
        this.arm = arm;
        this.intake = intake;

        addRequirements(arm, intake);
    }

    @Override
    public void initialize(){
        intake.isShooting = true;
        intake.runIntake(1);
        intake.runShooter(-1);
    }

    @Override
    public void execute(){
        armPosition = arm.getArmPosition();
        arm.runArm(arm.armPID.calculate(armPosition, ArmConstants.speakerEncoder));
    }

    @Override
    public void end(boolean isInterrupted){
        intake.isShooting = false;
        arm.runArm(0);
        intake.stop();
    }
}
