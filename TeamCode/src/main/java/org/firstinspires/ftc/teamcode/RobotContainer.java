package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotContainer {

    public final IntakeOuttakeSystem    intakeOuttake;
    public final TurretSystem           turret;
    public final HuskyLensCamera        husky = new HuskyLensCamera();
    private ElapsedTime                 timer = new ElapsedTime();

    //----------------------------------------------------------------------------------------------
    //Construction
    //----------------------------------------------------------------------------------------------

    public RobotContainer(HardwareMap hwMap){

        husky.init(hwMap);

        intakeOuttake = new IntakeOuttakeSystem(
                hwMap.get(DcMotor.class, "intakeMotor"),
                hwMap.get(DcMotor.class, "outtakeMotor"),
                hwMap.get(Servo.class, "launchServo")
        );

        turret = new TurretSystem(
                1,
                1,
                1,
                timer,
                husky,
                hwMap.get(DcMotor.class, "turretMotor")
        );
    }

}
