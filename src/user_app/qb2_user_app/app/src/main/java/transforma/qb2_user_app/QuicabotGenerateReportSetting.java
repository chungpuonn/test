package transforma.qb2_user_app;

import android.app.AlertDialog;
import android.app.ProgressDialog;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.text.TextUtils;
import android.util.Log;
import android.view.View;
import android.widget.EditText;
import android.widget.RadioGroup;
import android.widget.Toast;

import com.google.gson.Gson;

import ROS.*;
import transforma.qb2_user_app.rosmsg.*;

/**
 *This is an Activity class for report generation page, user are required to fill in relevant information before clicking the generate button
 */

public class QuicabotGenerateReportSetting extends QuicabotAppCompatActivity implements RosBridgeListener {


    private String postalCode;
    private String unitNumber;
    private String addressstring;
    private String workingModeCode;
    private String email;
    private int checkedbutton;
    private EditText unitNumberEditText;
    private EditText postalCodeEditText;
    private EditText emailText;
    private RadioGroup workingMode;
    private InspectionResult inspectionResult;
    private ProgressDialog reportGenDialog;


    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_quicabot_genrepport_setting);
        inspectionResult = InspectionResult.getInspectionResult();
        unitNumberEditText = findViewById(R.id.unitNumber);
        postalCodeEditText = findViewById(R.id.postalCode);
        emailText = findViewById(R.id.email);

        preferences = getSharedPreferences(PREFERENCE_NAME,MODE_PRIVATE);
        email = preferences.getString("emailString","lvzizhengde@gmail.com");
        emailText.setText(email);

//        if(InspectionResult.onGoingInspection()){
//            AlertDialog.Builder builder2 = new AlertDialog.Builder(this, R.style.MyDialogTheme) //An alert dialog about reconnection
//                    .setMessage("We find an existing inspection")
//                    .setPositiveButton("Enter", (dialog, which) -> {
//                        dialog.dismiss();
//                        Intent intent = new Intent(context, QuicabotInspectionMode.class);
//                        startActivity(intent);
//                        finish();
//                    });
//            builder2.show();
//        }
    }


    /**
     * Method called when start button is pressed, collecting all inspection setup information and pass it through RosBridge
     * @param view
     */
    public void genReport(View view) {


//        workingMode = findViewById(R.id.workingMode);
        unitNumber = unitNumberEditText.getText().toString();
        postalCode = postalCodeEditText.getText().toString();
        email = emailText.getText().toString();

        preferences = getSharedPreferences(PREFERENCE_NAME,MODE_PRIVATE);
        SharedPreferences.Editor editor = preferences.edit();  //Getting saved email address
        editor.putString("emailString", email);
        editor.apply();
        if (TextUtils.isEmpty(unitNumber) || TextUtils.isEmpty(postalCode) || TextUtils.isEmpty(email)) {//CHeck for empty input
            Toast.makeText(this, "Please enter correct information", Toast.LENGTH_SHORT).show();
        }
        else {
            if (unitNumber.contains("#")){ //Formalise the unit number by adding # in front

                unitNumber = unitNumber.replace("#", "Unit ");
                addressstring = unitNumber + ", S" + postalCode;
            } else {
                addressstring = "Unit " + unitNumber + ", S" + postalCode;
            }
            AlertDialog.Builder builder = new AlertDialog.Builder(this, R.style.MyDialogTheme) //An alert dialog about reconnection
                    .setMessage("Generating inspectionMode Result?")
                    .setPositiveButton("Start", (dialog, which) -> {
                        try {
                            reportGenDialog = new ProgressDialog(this);//A progress dialog showing connectino status
                            reportGenDialog.setMessage("Generating report, please wait");
                            reportGenDialog.show();
                            RosServiceMessageAddress messageAddress = RosServiceMessageAddress.buildRosSrvMessageAddress(CALL_SERVICE, SERVICE_ADDR, addressstring);
                            rosBridgeConnection.callService(messageAddress);
                        }catch (Exception e){
                            Log.i("Service Call Fail",e.getMessage());
                        }
                        dialog.dismiss();
                    })
                    .setNegativeButton(CANCEL, (dialog, which) -> dialog.dismiss());
            builder.show();
        }

    }

    /**
     * This method is a large onMessage method basically to update and display inspection information onto the screen
     * Case by case review is necessary
     * @param message Raw message string
     * @param rosMsgRaw rawMessage object with op field
     */
    @Override
    public void onMessage(String message, RosMsgRaw rosMsgRaw) {
        if (rosMsgRaw.getOp().equals(PUBLISH)){
            RosTopicMessage rosTopicMessage = new Gson().fromJson(message, RosTopicMessage.class);
            switch (rosTopicMessage.getTopic()){
//                case "666":
//                    break;
            }
        }else if (rosMsgRaw.getOp().equals(SERVICE_RESPONSE)){
            RosServiceMessage rosServiceMessage = new Gson().fromJson(message, RosServiceMessage.class);
            switch (rosServiceMessage.getService()){
                case SERVICE_ADDR:
                    if (/*rosServiceMessage.getResult()*/true){
                        RosServiceMessageGenReport genReport = RosServiceMessageGenReport.buildRosSrvMessageGenReport(CALL_SERVICE, SERVICE_REPORT_GEN, email);
                        rosBridgeConnection.callService(genReport);
                    }
                    else {
                        try {
                            reportGenDialog.dismiss();
                        } catch (Exception e) {
                            e.printStackTrace();
                        }
                        runOnUiThread(()-> Toast.makeText(context, "Sending request failed, please check service status", Toast.LENGTH_LONG).show());
                    }
                    break;
                case SERVICE_REPORT_GEN:
                    if (rosServiceMessage.getResult() && ((RosServiceMessageGenReport) rosServiceMessage).getValues().getIsGenerated() != 0) {
                        runOnUiThread(() -> Toast.makeText(context, "Report Generated!", Toast.LENGTH_LONG).show());
                        try {
                            reportGenDialog.dismiss();
                            inspectionResult.isReportedGenerated = true;
                            Thread.sleep(500);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                        onBackPressed();
                    } else {
                        runOnUiThread(() -> Toast.makeText(context, "Report Generation Failed!", Toast.LENGTH_LONG).show());
                        try {
                            reportGenDialog.dismiss();
                        } catch (Exception e) {
                            e.printStackTrace();
                        }

                    }
                    break;
            }
        }
    }
}
