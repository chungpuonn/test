package transforma.qb2_user_app;

import android.content.Intent;
import android.os.Build;
import android.os.Bundle;
import android.text.Layout;
import android.view.View;
import android.widget.TextView;

import ROS.RosMsgRaw;
import transforma.qb2_user_app.quicabotAppLicence.QuicabotAppLicencePage;

/**
 * This is an Activity Class of the about page of Transforma Robotics Company
 *
 * -When ever u use a 3rd party library remember to add the licence information to the licence page
 */

public class QuicabotAbout extends QuicabotAppCompatActivity {

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_quicabot_about);
        String versioncode = BuildConfig.VERSION_NAME;
        TextView view = findViewById(R.id.versionText);
        view.setText(getString(R.string.version, versioncode));
        TextView aboutUs = findViewById(R.id.aboutUsContent);
        TextView email = findViewById(R.id.emailContent);
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            aboutUs.setJustificationMode(Layout.JUSTIFICATION_MODE_INTER_WORD);
            email.setJustificationMode(Layout.JUSTIFICATION_MODE_INTER_WORD);
        }
    }

    /**
     * Method is called when RosBridge receives a message
     *
     * @param message
     * @param msgRaw
     */
    @Override
    public void onMessage(String message, RosMsgRaw msgRaw) {

    }

    public void phoneCall(View view) {
        Utils.phoneCall(this,ConfigurationParameters.tel);
    }

    public void writeEmail(View view) {
        Utils.sendEmail(this, ConfigurationParameters.email);
    }

    public void seeLicence(View view) {
        Intent intent = new Intent(context, QuicabotAppLicencePage.class);
        startActivity(intent);
    }
}
