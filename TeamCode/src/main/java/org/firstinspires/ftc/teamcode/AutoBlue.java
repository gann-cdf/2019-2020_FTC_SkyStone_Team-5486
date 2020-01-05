
package org.firstinspires.ftc.teamcode;

        import com.qualcomm.hardware.bosch.BNO055IMU;
        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.TouchSensor;
        import com.qualcomm.robotcore.util.ElapsedTime;
        import com.qualcomm.robotcore.hardware.Servo;

        import org.firstinspires.ftc.robotcore.external.ClassFactory;
        import org.firstinspires.ftc.robotcore.external.Telemetry;
        import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
        import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
        import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
        import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
        import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
        import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
        import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
        import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

        import java.util.ArrayList;
        import java.util.List;

        import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
        import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
        import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
        import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
        import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Autonomous(name = "AutoBlue", group = "Autonomous")

public class AutoBlue extends LinearOpMode {
    private DcMotor lf, rf, lr, rr;
    private Servo l_found, r_found;

    private ElapsedTime runtime = new ElapsedTime();

    //Numbers need to be fixed for all .setPower()!!
    public void stopMoving() {
        lf.setPower(0);
        lr.setPower(0);
        rf.setPower(0);
        rr.setPower(0);
    }

    public void forwardDrive() {
        lf.setPower(1);
        lr.setPower(1);
        rf.setPower(-1);
        rr.setPower(-1);
    }

    public void backwardDrive() {
        lf.setPower(1);
        lr.setPower(1);
        rf.setPower(-1);
        rr.setPower(-1);
    }

    public void strafeRight() {
        lf.setPower(1);
        lr.setPower(-1);
        rf.setPower(1);
        rr.setPower(-1);
    }

    public void strafeLeft() {
        lf.setPower(-1);
        lr.setPower(1);
        rf.setPower(-1);
        rr.setPower(1);
    }

    public void foundMoving(Boolean moveDown) {
        if (!moveDown) {
            l_found.setPosition(-0.7);
            r_found.setPosition(1.5);
        } else if (moveDown) {
            l_found.setPosition(0.6);
            r_found.setPosition(0.7);
        }
    }


    public void runOpMode() throws InterruptedException {
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            if (gamepad1.a){
                    forwardDrive();
                    stopMoving();
                    stop();}
            //moving the foundation into the building site
            else {
                forwardDrive();
                sleep (1000); //these sleep times are random guesses -- need to be tested
                stopMoving();
                foundMoving(true); //foundation mover thingies rotate perpendicular to the floor
                sleep(100);
                strafeRight();
                sleep (500);
                stopMoving();
                backwardDrive();
                sleep (1000);
                stopMoving();
                foundMoving(false); //foundation movers parallel to the floor


            //Can we use the claw yet??

            //move under skybridge
                strafeLeft();
                sleep(1000);
                stopMoving();
                stop();}
        }}
    }