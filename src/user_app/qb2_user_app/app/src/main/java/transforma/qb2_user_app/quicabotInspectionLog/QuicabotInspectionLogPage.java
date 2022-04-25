package transforma.qb2_user_app.quicabotInspectionLog;

import android.os.Bundle;
import androidx.recyclerview.widget.LinearLayoutManager;
import androidx.recyclerview.widget.RecyclerView;
import android.view.View;
import android.widget.TextView;

import com.google.gson.Gson;
import com.google.gson.reflect.TypeToken;

import java.lang.reflect.Type;
import java.util.ArrayList;

import ROS.RosMsgRaw;
import transforma.qb2_user_app.QuicabotAppCompatActivity;
import transforma.qb2_user_app.R;
import transforma.qb2_user_app.Utils;

/**
 *
 */

public class QuicabotInspectionLogPage extends QuicabotAppCompatActivity {


    private ArrayList<InspectionLog> inspectionLogList;
    private RecyclerView logRecyclerview;

    private TextView startDateTime, endDateTime, isReportGen;
    private TextView totalDefects, cracks, unevenness, misalisgn;
    private TextView JobCreated;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_quicabot_inspection_log_page);
        getInspectionLogList();
        startDateTime = findViewById(R.id.textStartTime);
        endDateTime = findViewById(R.id.textEndTime);
        isReportGen = findViewById(R.id.textIsReportGenerated);
        totalDefects = findViewById(R.id.textTotalDefects);
        cracks = findViewById(R.id.textCracks);
        unevenness = findViewById(R.id.textuneveness);
        misalisgn = findViewById(R.id.textMisalignment);
        JobCreated = findViewById(R.id.textJobCreated);

        logRecyclerview = findViewById(R.id.logRecyclerView);
        LogPageRecyclerviewAdapter myAdapter = new LogPageRecyclerviewAdapter(this,inspectionLogList);
        logRecyclerview.setLayoutManager(new LinearLayoutManager(this,LinearLayoutManager.VERTICAL, false));
        logRecyclerview.setAdapter(myAdapter);

    }



    @Override
    protected void onStop() {
        super.onStop();
        saveInspectionLogList(inspectionLogList);
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
    //TODO: ADD delete button
    public void moreActions(View view) {
        inspectionLogList.clear();
        LogPageRecyclerviewAdapter myAdapter = new LogPageRecyclerviewAdapter(this,inspectionLogList);
        logRecyclerview.setLayoutManager(new LinearLayoutManager(this,LinearLayoutManager.VERTICAL, false));
        logRecyclerview.setAdapter(myAdapter);
    }

    protected void showDetails(InspectionLog log){
        startDateTime.setText(log.getStartDateTime());
        endDateTime.setText(log.getEndDatetime());
        isReportGen.setText(log.isReportedGenerated() ? "Yes" : "No");
        totalDefects.setText(String.valueOf(log.getTotalDefects()));
        cracks.setText(String.valueOf(log.getTotalCracks()));
        unevenness.setText(String.valueOf(log.getTotalUnevenness()));
        misalisgn.setText(String.valueOf(log.getTotalMisalignment()));

        String jobLog = log.getJobCreatedandCreatedTime().toString();
        jobLog = jobLog.substring(1, jobLog.length()-1).replaceAll(",", "\n");
        JobCreated.setText(jobLog);

    }


    private ArrayList<InspectionLog> getInspectionLogList() {
        String logDataString = Utils.readFile(this, "log.txt");
        if (logDataString == null){
            inspectionLogList = new ArrayList<>();
        } else {
            Type inspectionLogListType = new TypeToken<ArrayList<InspectionLog>>(){}.getType();
            inspectionLogList = new Gson().fromJson(logDataString, inspectionLogListType);
        }
        return inspectionLogList;


    }
    private void saveInspectionLogList(ArrayList<InspectionLog> inspectionLogList) {
        String inspectionLogString = new Gson().toJson(inspectionLogList);
        Utils.writeFIle(this, inspectionLogString, "log.txt");
    }
}
