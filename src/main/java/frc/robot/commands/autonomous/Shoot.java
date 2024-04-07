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
    public void execute(){
        armPosition = arm.getArmPosition();
        // odometerX = RobotContainer.swerveSubsystem.odometer.getPoseMeters().getX();
        // diff = Math.abs(Robot.initialOdometerPose-odometerX);
        arm.runArm(Arm.armPID.calculate(armPosition, ArmConstants.speakerEncoder));
        
        intake.runIntake(1);
        intake.runShooter(-1);
        // } else {
        //     arm.runArm(0);
        //     intake.stop();

        //     end(false);
        // }
    }

    @Override
    public void end(boolean isInterrupted){
        arm.runArm(0);
        intake.stop();
    }
}
