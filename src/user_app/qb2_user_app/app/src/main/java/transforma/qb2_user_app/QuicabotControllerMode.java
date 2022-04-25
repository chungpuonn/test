package transforma.qb2_user_app;

import android.os.Bundle;
import android.os.Debug;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.Switch;
import android.widget.Toast;

import com.google.gson.Gson;
import com.xw.repo.BubbleSeekBar;

import ROS.RosMsgRaw;
import ROS.RosTopicMessage;
import io.github.controlwear.virtual.joystick.android.JoystickView;
import transforma.qb2_user_app.rosmsg.RosTopicMessageCmdVel;
import transforma.qb2_user_app.rosmsg.RosTopicMessageString;
import transforma.qb2_user_app.rosmsg.RosTopicMessagePanTiltCommand;
import transforma.qb2_user_app.rosmsg.RosTopicMessagePanTiltState;

/**
 *<p>This is an Activity Class of the robot base controller page. There are two elements in this page</p>
 *
 * <li>There is a pan tilt control system by two progress bar. It comes with a jog mode switch where data a send more frequently</li>
 * <li>There is a base movement control system by a virtual joystick. It comes with a switch to prevent accidental touch</li>
 */

public class QuicabotControllerMode extends QuicabotAppCompatActivity {

    private static final double VEL_MAXLINEAR = ConfigurationParameters.VEL_MAXLINEAR; //The maximum linear velocity of the robot moving system
    private static final double VEL_MAXANGULAR = ConfigurationParameters.VEL_MAXANGULAR; //The maximun rotation velocity of the robot moving system

    private static final String[] PAN_TILT_NAME = {"pan", "tilt"};
    private Switch baseMoveSwitch, jogModeSwitch;  //Switch that whether enable the progress bar
    private BubbleSeekBar panSeekbar, tiltSeekBar;
    private Button setPanTiltButton; // Manual set the system to current position when the jog mode is off
    private JoystickView myJoyStick;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_quicabot_controller_mode);


        jogModeSwitch = findViewById(R.id.jogModeEnable);
        baseMoveSwitch = findViewById(R.id.baseControlSwitch);
        panSeekbar = findViewById(R.id.panSeekBar);
        tiltSeekBar = findViewById(R.id.tiltSeekBar);
        setPanTiltButton = findViewById(R.id.setPanTilt);
        myJoyStick = findViewById(R.id.joystickView);

        //This set the listener of the joystick.
        myJoyStick.setOnMoveListener(this::moveTheRobot);

       //This set the listener of the pan seekbar for robot pan direction control
        panSeekbar.setOnProgressChangedListener(new BubbleSeekBar.OnProgressChangedListener() {
            @Override
            public void onProgressChanged(BubbleSeekBar bubbleSeekBar, int progress, float progressFloat, boolean fromUser) {
                if (jogModeSwitch.isChecked()) { setPanTilt(bubbleSeekBar) ; } }

            @Override
            public void getProgressOnActionUp(BubbleSeekBar bubbleSeekBar, int progress, float progressFloat) { }

            @Override
            public void getProgressOnFinally(BubbleSeekBar bubbleSeekBar, int progress, float progressFloat, boolean fromUser) {}

        });

        //This set the listener of the pan seekbar for robot pan direction control
        tiltSeekBar.setOnProgressChangedListener(new BubbleSeekBar.OnProgressChangedListener() {
            @Override
            public void onProgressChanged(BubbleSeekBar bubbleSeekBar, int progress, float progressFloat, boolean fromUser) {
                if (jogModeSwitch.isChecked()) { setPanTilt(bubbleSeekBar) ; }
            }
            @Override
            public void getProgressOnActionUp(BubbleSeekBar bubbleSeekBar, int progress, float progressFloat) {
            }
            @Override
            public void getProgressOnFinally(BubbleSeekBar bubbleSeekBar, int progress, float progressFloat, boolean fromUser) {
            }
        });

    }

    /**
     * This method is private method to be called for joystick listener
     * @param angle the displacement angle on the virtual joystick
     * @param strength the displacement extent on the virtual joystick
     */

    private void moveTheRobot(int angle, int strength) {
        double linear = Math.sin(Math.toRadians(angle)) * strength / 100 * VEL_MAXLINEAR;
        double angular = - Math.cos(Math.toRadians(angle)) * strength /100 * VEL_MAXANGULAR;

        RosTopicMessageCmdVel temp= RosTopicMessageCmdVel.buildRosTopicMessageCmdVel(PUBLISH, TOPIC_CMD_VEL, TYPE_GEOMETRY_VECTOR3, linear, angular);
        rosBridgeConnection.publish(temp);
    }

    /**
     * Method is called when RosBridge receives a message
     *
     * @param message
     * @param msgRaw
     */
    @Override
    public void onMessage(String message, RosMsgRaw msgRaw) {

        if (msgRaw.getOp().equals("publish")){
            RosTopicMessage rosTopicMessage = new Gson().fromJson(message, RosTopicMessage.class);
            //Use a switch case on
            switch (rosTopicMessage.getTopic()){
                case TOPIC_ROBOT_STATE:
                    final RosTopicMessageString robotState = new Gson().fromJson(message, RosTopicMessageString.class);
                    switch (robotState.getMsg().getData()){
                        case "initialising":
                            break;
                        case "idle":
                            //When the robot is ready we enable the control anain
                            if (!setPanTiltButton.isEnabled()){
                                runOnUiThread(()->{
                                    setPanTiltButton.setEnabled(true);
                                    setPanTiltButton.getBackground().setTint(getColor(R.color.colorSecondary));
                                });
                            }
                            break;
                        case "running":
                            //Disable the control when the robot is on operation
                            if (setPanTiltButton.isEnabled()){
                                runOnUiThread(()->{
                                    setPanTiltButton.setEnabled(false);
                                    setPanTiltButton.getBackground().setTint(getColor(R.color.colorPrimary));
                                });
                            }
                            break;
                        case "error":
                            runOnUiThread(()-> Toast.makeText(this, "Robot is in Error Status, Please Check Robot State", Toast.LENGTH_LONG).show());
                            Utils.deviceVibrate(this);
                            break;
                    }
                    break;
                case TOPIC_PAN_TILT_STATE:
                    Log.i("Pantilt State Received","1");
                    final RosTopicMessagePanTiltState pantileState = new Gson().fromJson(message, RosTopicMessagePanTiltState.class); //Deserialise the string
                    double panRad = pantileState.getMsg().getPosition()[0];
                    double tileRad = pantileState.getMsg().getPosition()[1];
                    double panDegreee = Math.toDegrees(panRad);
                    double tiltDedree = Math.toDegrees(tileRad);
                    panSeekbar.setProgress((float)(90.0 - panDegreee));
                    tiltSeekBar.setProgress((float)tiltDedree);
                    try {
                        rosBridgeConnection.unSubscribe(TOPIC_PAN_TILT_STATE, TYPE_SENSOR_MSGS_JOINT_STATE);
                    } catch (Exception e) {
                        e.printStackTrace();
                    }
            }
        }

    }

    /**
     * A switch to control whether the base movement controller is on/off
     * @param view
     */
    //TODO: Left for future development
    public void SwitchingControl(View view) {
        switch (view.getId()){
            case R.id.jogModeEnable:
                break;
            case R.id.baseControlSwitch:
                myJoyStick.setEnabled(baseMoveSwitch.isEnabled());
                break;
        }


    }

    /**
     * Set PanTilt system to the current position indicated by seek bar
     * @param view
     */
    public void setPanTilt(View view) {
        double pan = 90.0 - panSeekbar.getProgressFloat();
        double tilt = tiltSeekBar.getProgressFloat();
        double panRadian = Math.toRadians(pan);
        double tiltRadian = Math.toRadians(tilt);
        float panRound = (float)(Math.round(panRadian * 100.0) /100.0);
        float tiltRound = (float)(Math.round(tiltRadian * 100.0) /100.0);
        Log.i("PAN_TILT", Float.toString(panRound));
        Log.i("PAN_TILT", Float.toString(tiltRound));
        float[] panTiltArray = {panRound, tiltRound};
        try {
            RosTopicMessagePanTiltCommand msg = RosTopicMessagePanTiltCommand.buildRosTopicMessageTrajectoryMessageforPanTilt(
                    PUBLISH,
                    TOPIC_PAN_TILT_COMMAND,
                    TYPE_TRAJECTORYMSG,
                    PAN_TILT_NAME,
                    panTiltArray);

            rosBridgeConnection.publish(msg);
        } catch (Exception e) {
            //Log.i(ROS_BRIDGE_ERROR, e.getMessage());
            Toast.makeText(this, "Ros Bridge Error", Toast.LENGTH_SHORT).show();
        }
//        String toastMsg = String.format(Locale.getDefault(),"Pan-Tilt System set to %.2f and %.2f degrees", panSeekbar.getProgressFloat(), tiltSeekBar.getProgressFloat());
//        Toast.makeText(this, toastMsg, Toast.LENGTH_LONG).show();



    }

    /**
     * Reset PanTile system to default position
     * @param view
     */
    public void resetPanTilt(View view) {
        panSeekbar.setProgress(0);
        tiltSeekBar.setProgress(0);
        float[] panTiltArray = {(float)1.5708, (float)0.0};
        RosTopicMessagePanTiltCommand msg = RosTopicMessagePanTiltCommand.buildRosTopicMessageTrajectoryMessageforPanTilt(
                PUBLISH,
                TOPIC_PAN_TILT_COMMAND,
                TYPE_TRAJECTORYMSG,
                PAN_TILT_NAME,
                panTiltArray);

        rosBridgeConnection.publish(msg);
        Toast.makeText(this, "Pan-Tilt System reset to default position", Toast.LENGTH_SHORT).show();
    }
}
