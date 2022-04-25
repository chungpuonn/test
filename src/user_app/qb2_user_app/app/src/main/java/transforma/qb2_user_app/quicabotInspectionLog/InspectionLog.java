package transforma.qb2_user_app.quicabotInspectionLog;

import java.util.ArrayList;
import java.util.Date;
import java.util.HashMap;
import java.util.List;

/**
 * This is a json class for inspection logging, all variables are shown below
 *
 * Changing format of this class requires corresponding change to be made to the recyclerViewAdapter
 */

public class InspectionLog {

    public InspectionLog(String startDateTime, String endDatetime, ArrayList<String> jobCreatedandCreatedTime, int totalDefects, int totalCracks, int totalUnevenness, int totalMisalignment, boolean isReportedGenerated) {
        this.startDateTime = startDateTime;
        this.endDatetime = endDatetime;
        this.jobCreatedandCreatedTime = jobCreatedandCreatedTime;
        this.totalDefects = totalDefects;
        this.totalCracks = totalCracks;
        this.totalUnevenness = totalUnevenness;
        this.totalMisalignment = totalMisalignment;
        this.isReportedGenerated = isReportedGenerated;
    }


    public int getTotalDefects() {
        return totalDefects;
    }

    public int getTotalCracks() {
        return totalCracks;
    }

    public int getTotalUnevenness() {
        return totalUnevenness;
    }

    public int getTotalMisalignment() {
        return totalMisalignment;
    }

    public boolean isReportedGenerated() {
        return isReportedGenerated;
    }


    public void setTotalDefects(int totalDefects) {
        this.totalDefects = totalDefects;
    }

    public void setTotalCracks(int totalCracks) {
        this.totalCracks = totalCracks;
    }

    public void setTotalUnevenness(int totalUnevenness) {
        this.totalUnevenness = totalUnevenness;
    }

    public void setTotalMisalignment(int totalMisalignment) {
        this.totalMisalignment = totalMisalignment;
    }

    public void setReportedGenerated(boolean reportedGenerated) {
        isReportedGenerated = reportedGenerated;
    }

    public String getStartDateTime() {
        return startDateTime;
    }

    public String getEndDatetime() {
        return endDatetime;
    }

    public List<String> getJobCreatedandCreatedTime() {
        return jobCreatedandCreatedTime;
    }

    private String startDateTime, endDatetime;
    private List<String> jobCreatedandCreatedTime;
    private Integer totalDefects, totalCracks, totalUnevenness, totalMisalignment;
    private boolean isReportedGenerated;
}
