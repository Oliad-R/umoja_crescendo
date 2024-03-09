package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoShoot extends Command{
    Arm arm;
    Intake intake;
    SwerveSubsystem swerveSubsystem;
    int state = 0;

    public AutoShoot(SwerveSubsystem swerveSubsystem, Arm arm, Intake intake){
        this.arm = arm;
        this.intake = intake;
        this.swerveSubsystem = swerveSubsystem;
    }

    @Override
    public void execute(){
        System.out.println(state);
        if (state == 0) {
            if(!arm.getArmLimitSwitch()){
                System.out.println("MOVING ARM");
                arm.runArm(0.5);
                intake.stop();
            }
            else{
                System.out.println("resetting ARM");
                intake.stop();
                arm.resetEncoders();
                state++;
            }
        }
        else if(state==1){
            double armPosition = arm.rightEncoder.getPosition();
            if( armPosition > -23.5){
                arm.runArm(-0.5);
                intake.runShooter(1);
                intake.runIntake(0);
            }
            else {
                arm.runArm(0);
                state++;
            }
        }
        else if(state==2){
            intake.runIntake(1);
            intake.runShooter(-1);
        }
    }

}
