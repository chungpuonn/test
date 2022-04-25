package transforma.qb2_user_app;

import android.app.AlertDialog;
import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.os.Bundle;
import androidx.appcompat.app.AppCompatActivity;
import android.util.Log;
import android.widget.Toast;

import ROS.*;

/**
 * <p>A parent class for all activities that share some common attributes, all activity class are suggest to inherit this class.</p>
 *
 * <li>It inherits AppCompatActivity which is a standard parent class for all activities</li>
 * <li>It implements RosBRidgeListener and therefore it is able to react to all data received from RosBridge</li>
 */

public abstract class QuicabotAppCompatActivity extends AppCompatActivity implements RosBridgeListener {

    public static RosBridgeConnection rosBridgeConnection;
    Context context;
    SharedPreferences preferences;

    /**
     * get rosBridgeConnection onCreate
     * @param savedInstanceState
     */
    @Override
    protected void onCreate( Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        context = this;
        rosBridgeConnection = RosBridgeConnection.getRosBridgeConnection();
    }

    /**
     * Add listener onResume
     */

    @Override
    protected void onResume() {
        rosBridgeConnection.addListeners(this, this.getLocalClassName());
        super.onResume();

    }

    /**
     * Remove listener when onStop
     */

    @Override
    protected void onStop() {
        rosBridgeConnection.removeListeners(this);
        super.onStop();

    }

    /**
     * This is the onMessage Method responding to connection lost of the RosBridgeClient
     *
     * @param bool
     */
    @Override
    public void onMessage(boolean bool) {
        if (!bool) {
            runOnUiThread(() -> {
                AlertDialog.Builder builder = new AlertDialog.Builder(this, R.style.MyDialogTheme) //An alert dialog about reconnection
                        .setMessage("Connection Lost?")
                        .setPositiveButton("Reconnect", (dialog, which) -> {
                            dialog.dismiss();
                            rosBridgeConnection = RosBridgeConnection.getRosBridgeConnection();
                            if (rosBridgeConnection.isClosed())
                                rosBridgeConnection.reconnect();
                            rosBridgeConnection.addListeners(this,this.getLocalClassName());
                            rosBridgeConnection.subscribe(TOPIC_ROBOT_STATE,TYPE_STRING);
                            if (this.getLocalClassName().equals("QuicabotInspectionMode")){ //This is to prevent subscription lost when disconnected
                                try {
                                    rosBridgeConnection.subscribe(TOPIC_CRACK, TYPE_CRACK);
                                    rosBridgeConnection.subscribe(TOPIC_UNEVENMISALIGN, TYPE_UNEVENMISALIGN);
                                    rosBridgeConnection.subscribe(TOPIC_LIPPAGE, TYPE_LIPPAGE);
                                    rosBridgeConnection.subscribe(TOPIC_POINTCLOUD, TYPE_POINTCLOUD);
                                    rosBridgeConnection.removeListeners(this); //This is necessary as if the mainpage is not removed from the listener,
                                    // it is still able to pop up connection lost dialog when in background, this wil cause multiple dialog to pop out error
                                } catch (Exception e) {
                                    Log.i(ROS_BRIDGE_ERROR, e.getMessage());
                                    Toast.makeText(this, "Subscribed", Toast.LENGTH_SHORT).show();

                                }
                            }
                        })
                        .setNegativeButton(CANCEL, (dialog, which) -> {
                            dialog.dismiss();
                            Intent intent = new Intent(this, QuicabotLogin.class);
                            startActivity(intent);
                            finish();
                        });
                if (rosBridgeConnection.isClosed()) //This line prevents multi runnable thread in queue generating same dialog
                    builder.show();
            });
        }
    }







}
