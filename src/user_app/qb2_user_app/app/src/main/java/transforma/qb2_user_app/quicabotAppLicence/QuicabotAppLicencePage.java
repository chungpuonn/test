package transforma.qb2_user_app.quicabotAppLicence;

import androidx.appcompat.app.AppCompatActivity;
import android.os.Bundle;
import androidx.recyclerview.widget.LinearLayoutManager;
import androidx.recyclerview.widget.RecyclerView;

import java.util.ArrayList;
import java.util.List;

import transforma.qb2_user_app.R;

/**
 * This activity displays the 3rd party library used inside this project
 *
 * Format is fixed:
 * <p>Title</p>
 * <p>Description</p>
 * <p>Homepage Link</p>
 */
//TODO: Dont store the information in Code
public class QuicabotAppLicencePage extends AppCompatActivity {

    private static final String MIT_LICENCE = "MIT Licence";
    private static final String APACHE_LICENCE = "Apache Licence";

    List<AppLicenceUsed> licence;
    RecyclerView recyclerView;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_quicabot_licence_page);

        licence = new ArrayList<>();
        licence.add(new AppLicenceUsed(
                "Java-Websocket",
                "A barebones WebSocket client and server implementation written 100% in Java", MIT_LICENCE,
                "https://github.com/TooTallNate/Java-WebSocket"));
        licence.add(new AppLicenceUsed(
                "Gson",
                "Gson is a Java library that can be used to convert Java Objects into their JSON representation. It can also be used to convert a JSON string to an equivalent Java object. Gson can work with arbitrary Java objects including pre-existing objects that you do not have source-code of.", APACHE_LICENCE,
                "https://github.com/google/gson"));
        licence.add(new AppLicenceUsed(
                "virtual-joystick",
                "virtual-joystick-android provides a very simple and ready-to-use custom view which emulates a joystick for Android.", APACHE_LICENCE,
                "https://github.com/controlwear/virtual-joystick-android"));
        licence.add(new AppLicenceUsed(
                "BubbleSeekBar",
                "A beautiful Android custom seek bar, which has a bubble view with progress appearing upon when seeking. Highly customizable, mostly demands has been considered.", APACHE_LICENCE,
                "https://github.com/woxingxiao/BubbleSeekBar"));

        recyclerView = findViewById(R.id.licenceRecyclerView);
        LicencePageRecyclerViewAdapter myAdapter = new LicencePageRecyclerViewAdapter(this,licence);
        recyclerView.setLayoutManager(new LinearLayoutManager(this, LinearLayoutManager.HORIZONTAL, false));
        recyclerView.setAdapter(myAdapter);
    }

}
