package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeOuttakeSystem {

    private final DcMotor intakeMotor, outtakeMotor;
    private final Servo launchServo;

    private static final double CUSTOM_SERVO_EXTEND = 0.55;
    private static boolean isExtended = false;


    public IntakeOuttakeSystem(DcMotor intakeMotor, DcMotor outtakeMotor, Servo launchServo){

        this.intakeMotor = intakeMotor;
        this.outtakeMotor = outtakeMotor;

        this.outtakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.outtakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.launchServo = launchServo;
    }

    private void fullExtend(){ launchServo.setPosition(1.0);}
    private void customExtend(){ launchServo.setPosition(CUSTOM_SERVO_EXTEND);}
    private void close(){ launchServo.setPosition(0);}

    public void applyOuttakePower(double power, boolean isHeld) { outtakeMotor.setPower(isHeld ? power: 0); }

    public void applyIntakePower(double power, boolean isHeld){ intakeMotor.setPower(isHeld ? power: 0); }

    public void servoEvent(boolean isPressed){
        if (!isPressed){ return; }

        if (isExtended) {
            close();
            isExtended = false;
        } else{
            customExtend();
            isExtended = true;
        }
    }
}
