package transforma.qb2_user_app;

import android.annotation.SuppressLint;
import android.app.AlertDialog;
import android.app.ProgressDialog;
import android.content.Intent;
import android.os.Bundle;
import android.util.Log;
import android.util.Pair;
import android.view.MenuItem;
import android.view.View;
import android.webkit.WebView;
import android.widget.Button;
import android.widget.PopupMenu;
import android.widget.TextView;
import android.widget.Toast;

import com.google.gson.Gson;
import com.google.gson.reflect.TypeToken;

import java.lang.reflect.Type;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Timer;
import java.util.TimerTask;

import ROS.*;
import transforma.qb2_user_app.quicabotInspectionLog.InspectionLog;
import transforma.qb2_user_app.quicabotPointcloud.QuicabotPointCloudPage;
import transforma.qb2_user_app.rosmsg.*;

/**
 *This is an Activity class for inspection mode page, this page creates together with InspectionResult class singleton, which receives data from the robot and records the data temporarily.
 * <p>In this page, there exists:</p>
 * <li>4 buttons to control the inspection progress, pause, continue, stop, quit, when quit is pressed, a dialog would pop out to ask user whether to keep inspection in storage,
 * then the InspectionResult singleton would be destroyed and all data unrecorded will lost</li>
 *
 * <li>3 concurrent webView stacking together at same position displaying different defects live stream through port :8080.
 * A button is used to toggle webview transparency to see different webView. This is a brute force method as changing live stream address would have delay</li>
 *
 * <li>a button to view the 3d point cloud of the scanning result, fulfilled by OpenGl ES 3.0</li>
 *
 * <li>a data displaying penal showing live update of result, numbers are processed in InspectionResult singleton and displayed</li>
 */



public class QuicabotInspectionMode extends QuicabotAppCompatActivity implements RosBridgeListener {


    private InspectionResult inspectionResult;

    private String[] currentUrl;
    private ProgressDialog connectionDialog;
    private WebView imageWebViewCrack;
    private WebView imageWebViewUneven;
    private WebView imageWebViewLippage;
    private TextView imageTitle;
    private TextView totalDefectsNumber;
    private TextView crackNumber; // crackX, crackY;
    private TextView unevenNumber; // unevenX, unevenY;
    private TextView lippageNumber; // lippageX, lippageY;
    private TextView misalignNumber; // misalignX, misalignY;

    private String workingModeCode;
    private Button addingJobButton;

    private ArrayList<InspectionLog> inspectionLogList;

    private boolean pasueOrResueme = true;
    private Button pauseButton;


    @Override
    public void onCreate(Bundle savedInstanceState) {
        //Initialisation
        inspectionResult = InspectionResult.getInspectionResult();
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_quicabot_inspection_mode);

        //1.5sec delay created on purpose
        connectionDialog = new ProgressDialog(this);//A progress dialog showing connectino status
        connectionDialog.setCancelable(false);
        connectionDialog.setMessage("Initialising, please wait");
        connectionDialog.show();
        final Timer timer = new Timer();
        timer.schedule(new TimerTask() { //A connection failure would delay for 2 secs for smoother animation
            @Override
            public void run() {
                connectionDialog.dismiss();
            }
        }, 200);

        //inspectionMode Result Display Settings
        imageTitle = findViewById(R.id.imageStreamTitleText);
        crackNumber = findViewById(R.id.crackanddmgCount);
        unevenNumber = findViewById(R.id.unevenessCount);
        totalDefectsNumber = findViewById(R.id.totalDefectsCount);
        misalignNumber = findViewById(R.id.misAlignCount);
        lippageNumber = findViewById(R.id.lippageCount);
//        crackX = findViewById(R.id.crackanddmgX);
//        crackY = findViewById(R.id.crackanddmgY);
//        unevenX = findViewById(R.id.unevenessX);
//        unevenY = findViewById(R.id.unevenessY);
//        misalignX = findViewById(R.id.misAlignX);
//        misalignY = findViewById(R.id.misAlignY);
//        lippageX = findViewById(R.id.lippageX);
//        lippageY = findViewById(R.id.lippageY);
        addingJobButton = findViewById(R.id.buttonAddJob);
        pauseButton = findViewById(R.id.buttonPause);



        //TODO: Come up with a better logic to display images
        //We have three webViews stack up together as if we change image streaming address and change back, loss of previous pushed stream would result in blank and black webview

        currentUrl = new String[]{rosBridgeConnection.getImageStreamAddr() + URLIMAGECRACK, rosBridgeConnection.getImageStreamAddr() + URLIMAGEUNEVEN, rosBridgeConnection.getImageStreamAddr() + URLLIPPAGE};
        imageWebViewCrack = findViewById(R.id.imageStreamCrack);
        imageWebViewCrack.getSettings().setLoadWithOverviewMode(true);
        imageWebViewCrack.getSettings().setUseWideViewPort(true);
        imageWebViewCrack.getSettings().setDisplayZoomControls(true);
        imageWebViewCrack.getSettings().setBuiltInZoomControls(true);

        imageWebViewUneven = findViewById(R.id.imageStreamUneven);
        imageWebViewUneven.getSettings().setLoadWithOverviewMode(true);
        imageWebViewUneven.getSettings().setUseWideViewPort(true);
        imageWebViewUneven.getSettings().setDisplayZoomControls(true);
        imageWebViewUneven.getSettings().setBuiltInZoomControls(true);

        imageWebViewLippage = findViewById(R.id.imageStreamLippage);
        imageWebViewLippage.getSettings().setLoadWithOverviewMode(true);
        imageWebViewLippage.getSettings().setUseWideViewPort(true);
        imageWebViewLippage.getSettings().setDisplayZoomControls(true);
        imageWebViewLippage.getSettings().setBuiltInZoomControls(true);
        try {
            imageWebViewCrack.loadUrl(currentUrl[0]);
            imageWebViewUneven.loadUrl(currentUrl[1]);
            imageWebViewLippage.loadUrl(currentUrl[2]);
        } catch (Exception e) {
            e.printStackTrace();
            Toast.makeText(this, "WebView Error", Toast.LENGTH_SHORT).show();
        }


    }

    /**
     * Update Count numbers onResume every time
     */

    @SuppressLint("SetTextI18n")
    @Override
    protected void onResume() {
        totalDefectsNumber.setText(inspectionResult.totalDefectsCount.toString());
        crackNumber.setText(inspectionResult.crackCount.toString());
        unevenNumber.setText(inspectionResult.unevenCount.toString());
        misalignNumber.setText(inspectionResult.misalignCount.toString());
        lippageNumber.setText(inspectionResult.lippageCount.toString());
        try {
            imageWebViewCrack.loadUrl(currentUrl[0]);
            imageWebViewUneven.loadUrl(currentUrl[1]);
            imageWebViewLippage.loadUrl(currentUrl[2]);
        } catch (Exception e) {
            Log.i("WebView Problem", e.getMessage());
            Toast.makeText(this, "WebView Error", Toast.LENGTH_SHORT).show();
        }

        addingJobButton.setEnabled(!inspectionResult.isRobotWorking());
        addingJobButton.getBackground().setTint(getColor(!inspectionResult.isRobotWorking()? R.color.colorSecondary : R.color.colorPrimary));

        super.onResume();
    }


    /**
     * This method is a large onMessage method basically to update and display inspection information onto the screen
     * Case by case review is necessary
     *
     * @param message Raw message string
     * @param rosMsgRaw rawMessage object with op field
     */

    @SuppressLint("SetTextI18n")
    @Override
    public void onMessage(String message, RosMsgRaw rosMsgRaw) {
        if (rosMsgRaw.getOp().equals("publish")) {
            RosTopicMessage rosTopicMessage = new Gson().fromJson(message, RosTopicMessage.class);
            switch (rosTopicMessage.getTopic()) {
                case TOPIC_CRACK:
                    RosTopicMessageCrackResult rosMessageCrackResult = new Gson().fromJson(message, RosTopicMessageCrackResult.class);
                    if (rosMessageCrackResult.getMsg().getCrack_result() == 1) {
                        //final Pair<Integer, Integer> xyCrack = getXY(rosMessageCrackResult, true);
                        //runOnUiThread is necessary for updating the view
                        runOnUiThread(() -> {
                            //crackX.setText("X:" + xyCrack.first.toString());
                            //crackY.setText("Y:" + xyCrack.second.toString());
                            crackNumber.setText(inspectionResult.crackCount.toString());
                            totalDefectsNumber.setText(inspectionResult.totalDefectsCount.toString());
                        });
                        break;
                    }
                case TOPIC_UNEVENMISALIGN:
                    RosTopicMessageMisAlignLog rosMessageMisAlignLog = new Gson().fromJson(message, RosTopicMessageMisAlignLog.class);
                    if (rosMessageMisAlignLog.getMsg().getUneven_result() == 1) {
                        //final Pair<Integer, Integer> xyUneven = getXY(rosMessageMisAlignLog, true);
                        runOnUiThread(() -> {
                            //unevenX.setText("X:" + xyUneven.first.toString());
                            //unevenY.setText("Y:" + xyUneven.second.toString());
                            unevenNumber.setText(inspectionResult.unevenCount.toString());
                            totalDefectsNumber.setText(inspectionResult.totalDefectsCount.toString());
                        });
                    }
                    if (rosMessageMisAlignLog.getMsg().getMisalignment_result() == 1) {
                        //final Pair<Integer, Integer> xyMisAlign = getXY(rosMessageMisAlignLog, false);
                        runOnUiThread(() -> {
                            //misalignX.setText("X:" + xyMisAlign.first.toString());
                            //misalignY.setText("Y:" + xyMisAlign.second.toString());
                            misalignNumber.setText(inspectionResult.misalignCount.toString());
                            totalDefectsNumber.setText(inspectionResult.totalDefectsCount.toString());
                        });
                    }
                    break;
                case TOPIC_LIPPAGE:
                    RosTopicMessageLippageLog rosMessageLippageLog = new Gson().fromJson(message, RosTopicMessageLippageLog.class);
                    if (rosMessageLippageLog.getMsg().getLippage_result() == 1) {
                       // final Pair<Integer, Integer> xyLippage = getXY(rosMessageLippageLog, true);
                        runOnUiThread(() -> {
                           // lippageX.setText("X:" + xyLippage.first.toString());
                           // lippageY.setText("Y:" + xyLippage.second.toString());
                            lippageNumber.setText(inspectionResult.lippageCount.toString());
                            totalDefectsNumber.setText(inspectionResult.totalDefectsCount.toString());
                        });
                    }
                    break;
                case TOPIC_ROBOT_STATE:
                    final RosTopicMessageString robotState = new Gson().fromJson(message, RosTopicMessageString.class);
                    switch (robotState.getMsg().getData()){
                        case "initialising":
                            break;
                        case "idle":
                            inspectionResult.setRobotWorking(false);
                            if (!addingJobButton.isEnabled()) {
                                runOnUiThread(()-> {
                                    addingJobButton.setEnabled(true);
                                    addingJobButton.getBackground().setTint(getColor(R.color.colorSecondary));
                                });
                            }
                            break;
                        case "running":
                            inspectionResult.setRobotWorking(true);
                            if (addingJobButton.isEnabled()) {
                                runOnUiThread(()-> {
                                    addingJobButton.setEnabled(false);
                                    addingJobButton.getBackground().setTint(getColor(R.color.colorPrimary));
                                });
                            }
                            break;
                        case "error":
                            runOnUiThread(()-> Toast.makeText(context, "Robot is in Error Status, Please Check Robot State", Toast.LENGTH_LONG).show());
                            Utils.deviceVibrate(this);
                            break;
                    }
                    break;

            }
        } else if (rosMsgRaw.getOp().equals(SERVICE_RESPONSE) || rosMsgRaw.getOp().equals(CALL_SERVICE)) {
            RosServiceMessage rosServiceMessage = new Gson().fromJson(message, RosServiceMessage.class);
            switch (rosServiceMessage.getService()) {
                case SERVICE_REPORT_GEN:
                    break;
            }
        }
    }


    /**
     * This is a method that obtains location information contained in a rosMessage
     * Carefully add in all cases for all kind of RosTopicMessage received
     *
     * @param rosTopicMessage  the rosMsg received, used for inspection result reference
     * @param unevenOrMisAlign true for uneven false for others
     * @return a value pair containing the <x,y> information
     */

    private Pair<Integer, Integer> getXY(RosTopicMessage rosTopicMessage, boolean unevenOrMisAlign) {
        int a;
        int b;
        if (rosTopicMessage instanceof RosTopicMessageCrackResult) {
            a = (int) ((RosTopicMessageCrackResult) rosTopicMessage).getMsg().getCrack_loc().x;
            b = (int) ((RosTopicMessageCrackResult) rosTopicMessage).getMsg().getCrack_loc().y;
            return new Pair<>(a, b);
        } else if (rosTopicMessage instanceof RosTopicMessageMisAlignLog) {
            if (unevenOrMisAlign) {
                a = (int) ((RosTopicMessageMisAlignLog) rosTopicMessage).getMsg().getUneven_loc().x;
                b = (int) ((RosTopicMessageMisAlignLog) rosTopicMessage).getMsg().getUneven_loc().y;
                return new Pair<>(a, b);
            } else {
                a = (int) ((RosTopicMessageMisAlignLog) rosTopicMessage).getMsg().getMisalignment_loc().x;
                b = (int) ((RosTopicMessageMisAlignLog) rosTopicMessage).getMsg().getMisalignment_loc().y;
                return new Pair<>(a, b);
            }
        } else if (rosTopicMessage instanceof RosTopicMessageLippageLog) {
            a = (int) ((RosTopicMessageLippageLog) rosTopicMessage).getMsg().getLippage_loc().x;
            b = (int) ((RosTopicMessageLippageLog) rosTopicMessage).getMsg().getLippage_loc().y;
            return new Pair<>(a, b);
        }
        return new Pair<>(0, 0);
    }


    /**
     * Change the imagestream displayed when the flip is pressed
     *
     * @param view
     */
    public void FlipImage(View view) {
        PopupMenu popupMenu = new PopupMenu(this, view);
        popupMenu.setOnMenuItemClickListener((MenuItem item) -> {
            switch (item.getItemId()) {
                case R.id.displayCrack:
                    imageWebViewCrack.setVisibility(View.VISIBLE);
                    imageWebViewUneven.setVisibility(View.INVISIBLE);
                    imageWebViewLippage.setVisibility(View.INVISIBLE);
                    imageTitle.setText("Image of Crack:");
                    Toast.makeText(context, "Displaying Crack Image", Toast.LENGTH_SHORT).show();
                    break;
                case R.id.displayUnevenness:
                    imageWebViewCrack.setVisibility(View.INVISIBLE);
                    imageWebViewUneven.setVisibility(View.VISIBLE);
                    imageWebViewLippage.setVisibility(View.INVISIBLE);
                    imageTitle.setText("Image of Uneveness & Misalign:");
                    Toast.makeText(context, "Displaying Unevenness Image", Toast.LENGTH_SHORT).show();
                    break;
                case R.id.displayLippage:
                    imageWebViewCrack.setVisibility(View.INVISIBLE);
                    imageWebViewUneven.setVisibility(View.INVISIBLE);
                    imageWebViewLippage.setVisibility(View.VISIBLE);
                    imageTitle.setText("Image of Lippage:");
                    Toast.makeText(context, "Displaying Lippage Image", Toast.LENGTH_SHORT).show();
                    break;
            }
            return super.onContextItemSelected(item);
        });
        popupMenu.getMenuInflater().inflate(R.menu.web_view_selection_menu, popupMenu.getMenu());
        popupMenu.show();
    }


    /**
     * This is a method for button generating report. When this method called, it generates a alertDialog and if people choose yes,
     * it calls gen_report service attaching email address of the report receiver. While waiting for service response, a progress dialog would pop out.
     *
     * @param view
     */


    public void genReport(View view) {
        Intent intent = new Intent(context, QuicabotGenerateReportSetting.class);
        startActivity(intent);
    }


    /**
     * Method for Add Job button of the inspection mode. This method enables operator to add a inspection job to current job so that the defects counting are added
     *
     * @param view
     */
    public void addingInspectionJob(View view) {
        PopupMenu popupMenu = new PopupMenu(this, view);
        popupMenu.setOnMenuItemClickListener((MenuItem item) -> {
            switch (item.getItemId()) {
                case R.id.wallAuto:
                    workingModeCode = WORKING_MODE[0];
                    break;
                case R.id.wallCol:
                    workingModeCode = WORKING_MODE[1];
                    break;
                case R.id.single:
                    workingModeCode = WORKING_MODE[2];
                    break;
                case R.id.floor:
                    workingModeCode = WORKING_MODE[3];
            }
            try {
                rosBridgeConnection.advertise(TOPIC_OPMODE, TYPE_STRING);
                RosTopicMessageString msgString = RosTopicMessageString.buildRosTopicMessageString(PUBLISH, TOPIC_OPMODE, TYPE_STRING, workingModeCode);
                rosBridgeConnection.publish(msgString);
                inspectionResult.jobCreated.add(String.format("%s : %s", workingModeCode.toUpperCase(), Calendar.getInstance().getTime().toString()));
            } catch (Exception e) {
                Log.i(ROS_BRIDGE_ERROR, e.getMessage());
                Toast.makeText(this, "Ros Bridge Error", Toast.LENGTH_SHORT).show();
            }
            Toast.makeText(context, "New job started", Toast.LENGTH_SHORT).show();
            return true;
        });
        popupMenu.getMenuInflater().inflate(R.menu.inspection_mode_menu, popupMenu.getMenu());
        popupMenu.show();


    }


    /**
     * Method for continue button of the inspectionMode mode. This method publish a rosMsg of "resume"
     *
     * @param view
     */

    public void inspectionPause(View view) {
        try {
            rosBridgeConnection.advertise(TOPIC_COMMAND, TYPE_STRING);
            RosTopicMessage msg = RosTopicMessageString.buildRosTopicMessageString(PUBLISH, TOPIC_COMMAND, TYPE_STRING, pasueOrResueme? "pause" : "resume");
            rosBridgeConnection.publish(msg);
        } catch (Exception e) {
            Log.i("Advertising problem", e.getMessage());
            Toast.makeText(this, "Ros Bridge Error", Toast.LENGTH_SHORT).show();
        }
        pasueOrResueme = !pasueOrResueme;
        if (pasueOrResueme) {
            ((TextView)view).setText("PAUSE");
            Toast.makeText(this, "Inspection Continued", Toast.LENGTH_SHORT).show();
        } else {
            ((TextView)view).setText("RESUME");
            Toast.makeText(this, "Inspection Paused", Toast.LENGTH_SHORT).show();
        }

    }

    public void inspectionStop(View view) {
        try {
            rosBridgeConnection.advertise(TOPIC_COMMAND, TYPE_STRING);
            RosTopicMessage msg = RosTopicMessageString.buildRosTopicMessageString(PUBLISH, TOPIC_COMMAND, TYPE_STRING, "stop");
            rosBridgeConnection.publish(msg);
            inspectionResult.setRobotWorking(false);
            pauseButton.setText("PAUSE");
            if (!addingJobButton.isEnabled()) {
                addingJobButton.setEnabled(true);
                addingJobButton.getBackground().setTint(getColor(R.color.colorSecondary));
            }
        } catch (Exception e) {
            Log.i("Advertising problem", e.getMessage());
            Toast.makeText(this, "Ros Bridge Error", Toast.LENGTH_SHORT).show();
        }
        Toast.makeText(this, "Inspection Stop", Toast.LENGTH_SHORT).show();

    }

    /**
     * Method for Stop button of the inspection mode. This method null reference the inspection result for garbage collection, ends current activity and unsubscribe related topics
     *
     * @param view
     */
    public void inspectionQuit(View view) {
        if (!inspectionResult.jobCreated.isEmpty()) {
            AlertDialog.Builder builder = new AlertDialog.Builder(context, R.style.MyDialogTheme) //An alert dialog about reconnection
                    .setMessage("Keep this inspection in log?")
                    .setNegativeButton("No", (dialog, which) -> {
                        dialog.dismiss();
                        endInspection();
                    })
                    .setPositiveButton("Yes", ((dialog, which) -> {
                        try {
                            inspectionLogList = getInspectionLogList();
                            inspectionResult.logInspection(Calendar.getInstance().getTime(), inspectionLogList);
                            saveInspectionLogList(inspectionLogList);
                            //Log.i("Log Test Length", inspectionLogList.size() +"++++++++++++++++++++++++++++++++++++" );
                        } catch (Exception e) {
                            e.printStackTrace();
                        } finally {
                            dialog.dismiss();
                            endInspection();
                        }
                    }));
            builder.show();
        } else {
            endInspection();
        }
    }
    //TODO: SOLVE ORIENTATION CHANGE BUG
    private void endInspection(){
        inspectionResult.endInspection();
        try {
            rosBridgeConnection.unSubscribe(TOPIC_CRACK, TYPE_CRACK);
            rosBridgeConnection.unSubscribe(TOPIC_LIPPAGE, TYPE_LIPPAGE);
            rosBridgeConnection.unSubscribe(TOPIC_UNEVENMISALIGN, TYPE_UNEVENMISALIGN);
            rosBridgeConnection.unSubscribe(TOPIC_POINTCLOUD,TYPE_POINTCLOUD);
        } catch (Exception e) {
            Log.i(ROS_BRIDGE_ERROR, e.getMessage());
            Toast.makeText(this, ROS_BRIDGE_ERROR, Toast.LENGTH_SHORT).show();
        } finally {
            Toast.makeText(this, "Inspection Quit", Toast.LENGTH_SHORT).show();
            onBackPressed();
        }
    }

    private void saveInspectionLogList(ArrayList<InspectionLog> inspectionLogList) {
        String inspectionLogString = new Gson().toJson(inspectionLogList);
        Utils.writeFIle(this,inspectionLogString, "log.txt");
    }

    private ArrayList<InspectionLog> getInspectionLogList() {
        String logDataString = Utils.readFile(this,"log.txt");
        if (logDataString == null){
            inspectionLogList = new ArrayList<>();
        } else {
            Type inspectionLogListType = new TypeToken<ArrayList<InspectionLog>>(){}.getType();
            inspectionLogList = new Gson().fromJson(logDataString, inspectionLogListType);
        }
        return inspectionLogList;
    }


    public void generatePointCloud(View view) {
        AlertDialog.Builder builder = new AlertDialog.Builder(context, R.style.MyDialogTheme) //An alert dialog about reconnection
                .setMessage("Generate Point Cloud Model?")
                .setNegativeButton(CANCEL, (dialog, which) -> dialog.dismiss())
                .setPositiveButton("Start", ((dialog, which) -> {

                    if (inspectionResult.buffer == null) {
                        dialog.dismiss();
                        Toast.makeText(context, "No PointCloud Available", Toast.LENGTH_SHORT).show();

                    } else {
                        Intent intent = new Intent(context, QuicabotPointCloudPage.class);
                        startActivity(intent);
                        dialog.dismiss();
                    }

                }));
        builder.show();

    }

    public void viewInspectionSummary(View view) {
        
        AlertDialog.Builder builder = new AlertDialog.Builder(context, R.style.MyDialogTheme) //An alert dialog about reconnection
                .setMessage("View Inspection Summary")
                .setNegativeButton(CANCEL, (dialog, which) -> dialog.dismiss())
                .setPositiveButton("Start", ((dialog, which) -> {
                    Intent intent = new Intent(context, QuicabotInspectionResult.class);
                    startActivity(intent);
                    dialog.dismiss();

                }));
        builder.show();


    }

    public void controllerMode(View view) {

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
    }


    //TODO: Show notification when crack is detected
    //TODO: Store the crack image


}
