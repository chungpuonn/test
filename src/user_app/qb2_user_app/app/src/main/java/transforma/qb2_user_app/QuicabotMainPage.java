package transforma.qb2_user_app;


import android.Manifest;
import android.app.AlertDialog;
import android.content.Intent;
import android.content.pm.PackageManager;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;
import android.os.Bundle;
import android.util.Log;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;
import android.widget.PopupMenu;
import android.widget.Switch;
import android.widget.Toast;

import com.google.gson.Gson;

import ROS.*;
import transforma.qb2_user_app.quicabotInspectionLog.QuicabotInspectionLogPage;
import transforma.qb2_user_app.quicabotMap.QuicabotMapPage;
import transforma.qb2_user_app.rosmsg.InspectionResult;
import transforma.qb2_user_app.rosmsg.RosTopicMessageString;

/**
 * <p>
 * This is an activity class of the the mainPage of the quicabot interface. IN this page, u can control robot power switch and entering respective
 * mode to control the robot
 * </p>
 *
 * <p>
 * Procedure of publishing a rosmessage:
 * -Advertise the topic
 * -Publish a message
 * </p>
 *
 *<p>
 * Procedure of receiving a message:
 * -Advertise the topic
 * -Subscribe to the topic
 *</p>
 *
 */

public class QuicabotMainPage extends QuicabotAppCompatActivity implements RosBridgeListener {

    private Switch powerSwitch;
    private Button controllerModeButton, inspectionModeButton;
    private int permissionCheck;
    private String batteryStatus;
    private Button mapPageButton;
    //TODO: The app should be able to know whether the robot is ready for next stage(inspection )Via: robot status topic
    //TODO: On error notification

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_quicabot_main_page);


        /**
         * Check file writing permission
         */
        permissionCheck = ContextCompat.checkSelfPermission(this, Manifest.permission.WRITE_EXTERNAL_STORAGE);
        if (permissionCheck != PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.WRITE_EXTERNAL_STORAGE}, 123);
        }
        powerSwitch = findViewById(R.id.powerSwitch);
        controllerModeButton = findViewById(R.id.controllerMode);
        inspectionModeButton = findViewById(R.id.inspectionMode);
        mapPageButton = findViewById(R.id.mappingMode);


    }


    /**
     * A method that completely shuts down the robot
     * @param view
     */

    public void SwitchingPower(View view){
        if (!powerSwitch.isChecked()){
            AlertDialog.Builder builder = new AlertDialog.Builder(this, R.style.MyDialogTheme) //An alert dialog about reconnection
                    .setMessage("DO you want to shut down robot?")
                    .setPositiveButton("Yes", (dialog, which) -> {
                        try {
                            rosBridgeConnection.advertise(TOPIC_OPMODE, TYPE_STRING);
                            RosTopicMessage msg = RosTopicMessageString.buildRosTopicMessageString(PUBLISH, TOPIC_OPMODE, TYPE_STRING, "shutdown");
                            rosBridgeConnection.publish(msg);
                            rosBridgeConnection.client.close();
                        }catch (Exception e){
                            Log.i(ROS_BRIDGE_ERROR, e.getMessage());
                            Toast.makeText(this, "Ros Bridge Error", Toast.LENGTH_SHORT).show();
                        } finally {
                            dialog.dismiss();
                            finish();
                        }
                    })
                    .setCancelable(false)
                    .setNegativeButton(CANCEL, (dialog, which) -> {
                        powerSwitch.setChecked(true);
                        dialog.dismiss();
                    });
            builder.show();
        }
    }

    /**
     * A method when menu icon is pressed, displaying a toast message about battery status
     * @param view
     */
    public void ShowMenu(View view) {
        PopupMenu popupMenu = new PopupMenu(this, view);
        popupMenu.setOnMenuItemClickListener((MenuItem item) ->{
            switch (item.getItemId()){
                case R.id.reboot:
                    AlertDialog.Builder builder = new AlertDialog.Builder(this, R.style.MyDialogTheme) //An alert dialog about reconnection
                            .setMessage("Do you want to reboot robot?")
                            .setPositiveButton("Yes", (dialog, which) -> {
                                try {
                                    rosBridgeConnection.advertise(TOPIC_OPMODE, TYPE_STRING);
                                    RosTopicMessage msg = RosTopicMessageString.buildRosTopicMessageString(PUBLISH, TOPIC_OPMODE, TYPE_STRING, "reboot");
                                    rosBridgeConnection.publish(msg);
                                    rosBridgeConnection.close();
                                }catch (Exception e){
                                    Log.i(ROS_BRIDGE_ERROR, e.getMessage());
                                    Toast.makeText(this, "Ros Bridge Error", Toast.LENGTH_SHORT).show();
                                } finally {
                                    dialog.dismiss();
                                    Intent intent = new Intent(context, QuicabotLogin.class);
                                    startActivity(intent);
                                    finish();
                                }
                            })
                            .setNegativeButton(CANCEL, (dialog, which) -> dialog.dismiss());
                    builder.show();
                    break;
                case R.id.about:
                    Intent intentAbout = new Intent(context, QuicabotAbout.class);
                    startActivity(intentAbout);
                    break;
                case R.id.log:
                    Intent intentLog = new Intent(context, QuicabotInspectionLogPage.class);
                    startActivity(intentLog);
                    break;
            }
            return onContextItemSelected(item);
        });
        popupMenu.getMenuInflater().inflate(R.menu.mainpage_more_menu,popupMenu.getMenu());
        popupMenu.show();

    }


    /**
     * A method when battery icon is pressed, displaying a toast message about battery status
     * @param view
     */

    //TODO：This method is left for future development
    public void showBattery(View view) {
        Toast.makeText(this, "Battery Chagring Status: Very Good", Toast.LENGTH_SHORT).show();

    }

    @Override

    public void onMessage(String message, RosMsgRaw rosMsgRaw) {
        if (rosMsgRaw.getOp().equals("publish")){
            RosTopicMessage rosTopicMessage = new Gson().fromJson(message, RosTopicMessage.class);
            //Use a switch case on
            switch (rosTopicMessage.getTopic()) {
                case TOPIC_ROBOT_STATE:
                    final RosTopicMessageString robotState = new Gson().fromJson(message, RosTopicMessageString.class);
                    switch (robotState.getMsg().getData()) {
                        case "initialising":
                            if (controllerModeButton.isEnabled()) {
                                runOnUiThread(() -> {
                                    Toast.makeText(context, "Robot is initialising", Toast.LENGTH_SHORT).show();
                                    controllerModeButton.setEnabled(false);
                                    controllerModeButton.getBackground().setTint(getColor(R.color.colorPrimary));
                                    inspectionModeButton.setEnabled(false);
                                    inspectionModeButton.getBackground().setTint(getColor(R.color.colorPrimary));
                                });
                            }
                            break;
                        case "idle":
                            if (!controllerModeButton.isEnabled()) {
                                runOnUiThread(() -> {
                                    controllerModeButton.setEnabled(true);
                                    controllerModeButton.getBackground().setTint(getColor(R.color.colorSecondary));
                                    inspectionModeButton.setEnabled(true);
                                    inspectionModeButton.getBackground().setTint(getColor(R.color.colorSecondary));
                                });
                            }
                            break;
                        case "running":
                            break;
                        case "error":
                            runOnUiThread(() -> Toast.makeText(context, "Robot is in Error Status, Please Check Robot State", Toast.LENGTH_LONG).show());
                            //Vibrate when there is an error
                            Utils.deviceVibrate(this);
                            break;
                    }
                    break;

            }
        }

    }


    /**
     * A reminder when back is pressed
     */
    @Override
    public void onBackPressed() {
        AlertDialog.Builder builder = new AlertDialog.Builder(this, R.style.MyDialogTheme) //An alert dialog about reconnection
                .setMessage("Do you want to quit application? \n(Close Connection)")
                .setPositiveButton("Yes", (dialog, which) -> {
                    try {
                        rosBridgeConnection.close();
                    } catch (Exception e) {
                        e.printStackTrace();
                        Toast.makeText(this, "Ros Bridge Error", Toast.LENGTH_SHORT).show();
                    } finally {
                        dialog.dismiss();
                        finish();
                    }
                })
                .setNegativeButton(CANCEL, (dialog, which) -> dialog.dismiss());
        builder.show();
    }


    /**
     * Depends on which button pressed, the robot:
     * -Enters Controller Mode, which controls the movement of the vehicle or
     * -Enters inspectionMode mode, which displays and inspection result or control inspection progress
     * @param view
     */

    public void workingMode(View view) {
        switch (view.getId()){

            case R.id.controllerMode:
                try {
                    //rosBridgeConnection.subscribe(TOPIC_CRACK, TYPE_CRACK);
                    rosBridgeConnection.advertise(TOPIC_PAN_TILT_COMMAND, TYPE_TRAJECTORYMSG);
                    rosBridgeConnection.subscribe(TOPIC_PAN_TILT_COMMAND, TYPE_TRAJECTORYMSG);
                    rosBridgeConnection.advertise(TOPIC_PAN_TILT_STATE, TYPE_SENSOR_MSGS_JOINT_STATE);
                    rosBridgeConnection.subscribe(TOPIC_PAN_TILT_STATE, TYPE_SENSOR_MSGS_JOINT_STATE);
                    rosBridgeConnection.advertise(TOPIC_CMD_VEL, TYPE_GEOMETRY_VECTOR3);
                    Intent intent = new Intent(this, QuicabotControllerMode.class);
                    startActivity(intent);
                } catch (Exception e) {
                    Log.i(ROS_BRIDGE_ERROR, e.getMessage());
                    Toast.makeText(this, "Ros Bridge Error", Toast.LENGTH_SHORT).show();

                }
                break;
            case R.id.inspectionMode:

                try {
                    if(! InspectionResult.onGoingInspection()){
                        rosBridgeConnection.subscribe(TOPIC_CRACK, TYPE_CRACK);
                        rosBridgeConnection.subscribe(TOPIC_UNEVENMISALIGN, TYPE_UNEVENMISALIGN);
                        rosBridgeConnection.subscribe(TOPIC_LIPPAGE, TYPE_LIPPAGE);
                        rosBridgeConnection.subscribe(TOPIC_POINTCLOUD, TYPE_POINTCLOUD);
                    }
                    /**
                     * This section does the subscription to certain topic
                     * The subscription is done here to prevent data lost
                     */
                    Intent intent = new Intent(this, QuicabotInspectionMode.class);
                    startActivity(intent);
                } catch (Exception e) {
                    Log.i(ROS_BRIDGE_ERROR, e.getMessage());
                    Toast.makeText(this, "Ros Bridge Error", Toast.LENGTH_SHORT).show();

                }
                break;
            case R.id.mappingMode:
                try {
                    rosBridgeConnection.subscribe(TOPIC_MAP, TYPE_NAVMSGS_OCCUPANCYGRID);
                    rosBridgeConnection.subscribe(TOPIC_POSEUPDATE, TYPE_GEOMETRY_POSEWITHCOVARIANCESTAMPED);
                    rosBridgeConnection.advertise(TOPIC_CMD_VEL, TYPE_GEOMETRY_VECTOR3);
                    Intent intent = new Intent(this, QuicabotMapPage.class);
                    startActivity(intent);
                } catch (Exception e) {
                    Log.i(ROS_BRIDGE_ERROR, e.getMessage());
                    Toast.makeText(this, "Ros Bridge Error", Toast.LENGTH_SHORT).show();
                }
                break;
        }
    }

}
