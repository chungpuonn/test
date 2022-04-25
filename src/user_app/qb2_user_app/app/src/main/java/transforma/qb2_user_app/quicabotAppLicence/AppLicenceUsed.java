package transforma.qb2_user_app.quicabotAppLicence;


/**
 * Json Object class containing information about the 3rd party library we used for this project
 * Changing format of this class requires corresponding change to be made to the recyclerViewAdapter
 */

public class AppLicenceUsed {
    private String title;
    private String content;
    private String type;
    private String homepage;

    public AppLicenceUsed() {
    }

    public AppLicenceUsed(String title, String content, String type, String homepage) {
        this.title = title;
        this.content = content;
        this.type = type;
        this.homepage = homepage;
    }

    public String getTitle() {
        return title;
    }

    public String getContent() {
        return content;
    }

    public String getType() {
        return type;
    }

    public String getHomepage() {
        return homepage;
    }

    public void setTitle(String title) {
        this.title = title;
    }

    public void setContent(String content) {
        this.content = content;
    }

    public void setType(String type) {
        this.type = type;
    }

    public void setHomepage(String homepage) {
        this.homepage = homepage;
    }
}
