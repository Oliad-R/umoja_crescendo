package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoShoot extends Command{
    Arm arm;
    Intake intake;
    SwerveJoystick sj;
    int state = 0;

    public final class AutoShootState {
        public static final int RESET_ARM = 0;
        public static final int AIM = 1;
        public static final int SHOOT = 2;
        
    }
    Timer timer;
    double startTime, currentTime;
    PIDController armPID = new PIDController(ArmConstants.kP, 0, 0.01);

    public AutoShoot(Arm arm, Intake intake, int state){
        this.arm = arm;
        this.intake = intake;
        this.state = state;

        addRequirements(arm, intake);
    }

    public AutoShoot(Arm arm, Intake intake){
        this.arm = arm;
        this.intake = intake;

        addRequirements(arm, intake);
    }

    @Override
    public void execute(){
        double armPosition = arm.rightEncoder.getPosition();
        SmartDashboard.putNumber("ARM ENCODER", armPosition);
        if (state == AutoShootState.RESET_ARM) {
            if(!arm.getArmLimitSwitch()){
                arm.runArm(0.5);
                intake.stop();
            }
            else{
                intake.stop();
                arm.resetEncoders();
                state++;
            }
        }
        else if(state == AutoShootState.AIM){
            if(Math.abs(armPosition-ArmConstants.speakerEncoder) > 0.1){
                // arm.runArm(-0.5);
                arm.runArm(armPID.calculate(armPosition, ArmConstants.speakerEncoder));
                intake.runShooter(1);
                intake.runIntake(0);
            }
            else {
                state++;
                startTime = timer.getFPGATimestamp();
            }
        }
        else if(state == AutoShootState.SHOOT){
            currentTime = timer.getFPGATimestamp();
            if (currentTime-startTime < 3){
                arm.runArm(armPID.calculate(armPosition, ArmConstants.speakerEncoder));
                intake.runIntake(1);
                intake.runShooter(-1);
            } else {
                arm.runArm(0);
                intake.stop();
                state++;
            }
        } else if (state == 3){
            if(!arm.getArmLimitSwitch()){
                arm.runArm(0.2*armPID.calculate(armPosition, 0));
            } else {
                arm.runArm(0);
                // intake.runIntake(0.4);
                state++;
                end(true);
            }
        }
    }

    public boolean isFinished(){
        return state==4;
    }

}
