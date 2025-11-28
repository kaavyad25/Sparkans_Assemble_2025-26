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

    //TODO: Tune Kp, Kd, Ki
    private final double    Kp = 1.0;
    private final double    Kd = 1.0;
    private final double    Ki = 1.0;

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
                Kp,
                Ki,
                Kd,
                timer,
                husky,
                hwMap.get(DcMotor.class, "turretMotor")
        );
    }

}
