package transforma.qb2_user_app;

import ROS.*;
import android.app.AlertDialog;
import android.app.ProgressDialog;
import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.net.wifi.WifiManager;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.EditText;
import android.widget.Toast;


/**
 * This is an Activity class of the Login Page, it displays text field for people to connect to the robot
 *
 * <p>It initialises the rosbridge connection</p>
 * <p>It subscribe to robot status topic to know robot current state</p>
 *
 *
 *
 */
public class QuicabotLogin extends QuicabotAppCompatActivity implements RosBridgeListener {


    private ProgressDialog connectionDialog;
    private String RobotId;
    private String password;
    private EditText robotId;
    private EditText connectPassword;

    /**
     * QuicabotLogin screen onCreate would initiate all view objects
     * @param savedInstanceState
     */


    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_login);
        robotId = findViewById(R.id.RobotId);
        connectPassword = findViewById(R.id.Password);

        //Read information from previous login
        preferences = getSharedPreferences(PREFERENCE_NAME,MODE_PRIVATE);
        RobotId = preferences.getString("robotID", "168.0.130");
        robotId.setText(RobotId);
    }
    /**
     * A method specifying what happens when the connect button is pressed
     * @param view
     */
    public void Connect(final View view){
        connectionDialog = new ProgressDialog(this);//A progress dialog showing connectino status
        connectionDialog.setMessage("Connecting, please wait");
        connectionDialog.show();
        connectionDialog.setCancelable(false);


        //This helps to avoid connection service delay from the beginning as the connection service may not issue an error when wifi is not connected
        if(!((WifiManager) getApplicationContext().getSystemService(Context.WIFI_SERVICE)).isWifiEnabled()){
            onMessage(false);
            return;
        }

        RobotId = robotId.getText().toString();
        password = connectPassword.getText().toString();

        //TODO: Password is left for future development

        if (password.equals("1"))
        {
            Intent intent = new Intent(this, QuicabotMainPage.class);
            connectionDialog.dismiss();
            startActivity(intent);
            finish();
            return;
        }

        //Store information for next login
        preferences = getSharedPreferences(PREFERENCE_NAME,MODE_PRIVATE);
        SharedPreferences.Editor editor = preferences.edit();
        editor.putString("robotID", RobotId);
        editor.apply();

        rosBridgeConnection.client = null; // This is necessary to reset client parameters
        rosBridgeConnection.setImageStreamAddr(HTTPSTART+RobotId+IMAGEPORT);
        rosBridgeConnection.initSocket(SOCKETSTART+RobotId+STRINGPORT);//Starting Connection

    }

    /**
     * A method that creates an alert dialog on failure and enters new page on connection
     * @param bool
     */
    @Override
    public void onMessage(boolean bool) {

        final Context context = this;
        if (bool) {
            rosBridgeConnection.subscribe(TOPIC_ROBOT_STATE, TYPE_STRING);
            Intent intent = new Intent(this, QuicabotMainPage.class);
            connectionDialog.dismiss();
            startActivity(intent);


            finish();
        } else {
            runOnUiThread(()->{
                connectionDialog.dismiss();
                AlertDialog.Builder builder = new AlertDialog.Builder(context, R.style.MyDialogTheme) //An alert dialog about reconnection
                        .setMessage("Connection Failed, please check Wifi connection or Robot ID")
                        .setNegativeButton(CANCEL, (dialog, which) -> dialog.dismiss());
                builder.show();
            });
        }
    }

    @Override
    public void onMessage(String message, RosMsgRaw msgRaw) {
        return;
    }

    /**
     * Quite App directly onBackPressed
     */

    @Override
    public void onBackPressed() {
        finishAffinity();
    }
}
