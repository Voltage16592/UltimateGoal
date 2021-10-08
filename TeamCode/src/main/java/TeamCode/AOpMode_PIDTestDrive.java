package TeamCode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AOpMode_PIDTestDrive", group="Linear Opmode")
//@Disabled
public class AOpMode_PIDTestDrive extends LinearOpMode {

    /* Declare OpMode members. */

    private ElapsedTime runtime = new ElapsedTime();
    private SubSys_MecDrive mecDrive = new SubSys_MecDrive();
    private Subsys_gyroscope gyroscope = new Subsys_gyroscope();
    private double outputScale = 1;
    private double speed = 0.5;


    @Override
    public void runOpMode() {

        mecDrive.init(hardwareMap);
        gyroscope.init(hardwareMap);
        telemetry.addData("Status", "Initialized");        // Send telemetry message to signify robot waiting;
        telemetry.update();

        waitForStart();     // Wait for the game to start (driver presses PLAY)

        //write code here
        encoderDrive(50, 30);


    }

    public void updateTelemetryStatus(int num){
        telemetry.addData("status", num);
        telemetry.update();
        //sleep(1000);
    }


    public void encoderDrive(double inches, double timeoutS) {
        int targetEncoderPosition = (int) (inches * mecDrive.COUNTS_PER_INCH * mecDrive.Adjust);

        gyroscope.resetAngle();

        double k_p = 0.01;
        double k_i = 0;
        double k_d = 0;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            resetAllMotors();

            double int_error = 0;
            double prev_error = 0;
            int maththingy = Math.max(Math.max(mecDrive.fleft_drive.getCurrentPosition(), mecDrive.fright_drive.getCurrentPosition()), Math.max(mecDrive.bleft_drive.getCurrentPosition(), mecDrive.bright_drive.getCurrentPosition()));
            telemetry.addLine(String.valueOf(maththingy));
            telemetry.update();
            setPower(0, 1);
            sleep(2000);
            telemetry.addLine("angle"+ gyroscope.getAngle());
            telemetry.update();
            while (opModeIsActive() /*&& Math.max(Math.max(mecDrive.fleft_drive.getCurrentPosition(), mecDrive.fright_drive.getCurrentPosition()), Math.max(mecDrive.bleft_drive.getCurrentPosition(), mecDrive.bright_drive.getCurrentPosition()))<targetEncoderPosition*/ && runtime.seconds() < timeoutS){
                double error = gyroscope.getAngle();
                double output = k_p*error + k_i*int_error + k_d*(error-prev_error);
                int direction = (int) (-error/Math.abs(error));
                telemetry.addLine("direction: "+direction);
                telemetry.addLine("error: "+error);
                telemetry.addLine("prev-error: "+prev_error);
                telemetry.addLine("output: " + output);
                telemetry.update();
                setPower(output, direction);
                prev_error = error;

            }
            stopAllMotors();

        }

    }

    private void resetAllMotors(){
        mecDrive.fleft_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mecDrive.fright_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mecDrive.bleft_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mecDrive.bright_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        mecDrive.fleft_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mecDrive.fright_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mecDrive.bleft_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mecDrive.bright_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void stopAllMotors(){
        mecDrive.fleft_drive.setPower(0);
        mecDrive.fright_drive.setPower(0);
        mecDrive.bleft_drive.setPower(0);
        mecDrive.bright_drive.setPower(0);

        mecDrive.fleft_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mecDrive.fright_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mecDrive.bleft_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mecDrive.bright_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setPower(double output, int direction){
        mecDrive.fleft_drive.setPower(speed+output*direction);
        mecDrive.fright_drive.setPower(speed-output*direction);
        mecDrive.bleft_drive.setPower(speed+output*direction);
        mecDrive.bright_drive.setPower(speed-output*direction);
    }

}