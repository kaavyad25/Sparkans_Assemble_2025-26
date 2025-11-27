package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Main TeleOp Handler.
 */


public class AssembledBot extends LinearOpMode {
    ElapsedTime     elapsedTime = new ElapsedTime();
    DriveTrain      dt = new DriveTrain();
    IMUOdometry     imu = new IMUOdometry();
    HuskyLensCamera husky = new HuskyLensCamera();

    // Placeholders
    //TODO: Manually tune Kp, Ki, Kd
    RobotFunctions  func = new RobotFunctions(1.0, 1.0, 1.0, husky, elapsedTime);

    private static final double     STANDARD_OUTTAKE_POWER = 1.0;  /** Change later when tested*/
    private static final double     STANDARD_INTAKE_POWER = 1.0;  /** Change later when tested*/
    private static double           driveScalar = 1.0; /** will change with controller*/
    private int                     tagId;
    private boolean                 isTagRead = false;


    public void runOpMode(){

        dt.init(hardwareMap);
        func.init(hardwareMap);
        imu.init(hardwareMap);
        husky.init(hardwareMap);

        waitForStart();

        dt.isMoving = true;


        while (opModeIsActive()) {

            double  targetPowerX = gamepad1.left_stick_x;
            double  targetPowerY = gamepad1.left_stick_y;
            double  targetRotation = gamepad1.right_stick_x;

            boolean intakeButton =  gamepad1.left_bumper;
            boolean outtakeButton = gamepad1.right_bumper;
            boolean servoTrigger =  gamepad1.a;
            boolean decrementDriveSpeed = gamepad1.dpad_down;
            boolean incrementDriveSpeed = gamepad1.dpad_up;

            double  currentHeadingDeg = imu.getContinuousHeadingDeg();

            func.activateIntake(STANDARD_INTAKE_POWER, intakeButton);
            func.activateOuttake(STANDARD_OUTTAKE_POWER, outtakeButton);
            func.servoEvent(servoTrigger);

            // small adjustment to drive speed,
            if (decrementDriveSpeed) driveScalar -= 0.1;
            if (incrementDriveSpeed) driveScalar += 0.1;

            driveScalar = Range.clip(driveScalar, 0.0, 1.0);

            dt.setSpeedScalar(driveScalar);
            dt.fieldOrientedTranslate(targetPowerX, targetPowerY, targetRotation, currentHeadingDeg);

            // if camera is serving other purpose, like colour detection, tag recognition cannot take place

            if (husky.isTagRecognition()){

                func.continuousUpdate();

                if (!isTagRead) {

                    this.tagId = husky.getTagID();
                    isTagRead = true;
                }
            }

            if (servoTrigger) func.isExtended = !func.isExtended; // see RobotFunctions.servoEvent()

        }
    }
}