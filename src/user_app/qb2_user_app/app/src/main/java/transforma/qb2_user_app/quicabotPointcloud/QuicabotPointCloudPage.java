package transforma.qb2_user_app.quicabotPointcloud;

import android.os.Bundle;
import android.util.DisplayMetrics;
import android.view.WindowManager;

import ROS.RosMsgRaw;
import transforma.qb2_user_app.QuicabotAppCompatActivity;
import transforma.qb2_user_app.R;

/**
 * This activity is to display raw pointCloud data received from the rosBridge. The rosBridge would transfer around 30k to 90k points float to our client
 * This activity basically use OpenGL ES 3.2 to render white pointCloud in a 3D interactive view for users to view and inspect raw data.
 *
 * The interaction includes model rotation, translation and zooming.
 * Rendering is done using GPU graphic memory in order to improve performance and reduce memory usage.
 */

public class QuicabotPointCloudPage extends QuicabotAppCompatActivity {

    private PointCloudView pointCloudView;
    PointCloudRenderer renderer;


    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN, WindowManager.LayoutParams.FLAG_FULLSCREEN);

        final DisplayMetrics displayMetrics = new DisplayMetrics();
        getWindowManager().getDefaultDisplay().getMetrics(displayMetrics);

//        pointCloudView = new PointCloudView(this, displayMetrics.density);
        setContentView(R.layout.activity_quicabot_pointcloudview);
        pointCloudView = findViewById(R.id.pointCloudView);
        renderer = new PointCloudRenderer(this);
        pointCloudView.init(renderer, displayMetrics.density);



    }

    @Override
    protected void onResume()
    {
        // The activity must call the GL surface view's onResume() on activity onResume().
        super.onResume();
        pointCloudView.onResume();
    }

    @Override
    protected void onPause()
    {
        // The activity must call the GL surface view's onPause() on activity onPause().
        super.onPause();
        pointCloudView.onPause();
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



}