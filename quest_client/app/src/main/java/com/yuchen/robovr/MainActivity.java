package com.yuchen.robovr;

import android.app.Activity;
import android.os.Bundle;
import android.widget.TextView;

public final class MainActivity extends Activity {
    static {
        System.loadLibrary("robovr_quest_client");
    }

    private TextView statusView;

    private static native void nativeStart(MainActivity activity, String host, int port);
    private static native void nativeStop();
    private static native void nativeOnResume();
    private static native void nativeOnPause();

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        statusView = new TextView(this);
        statusView.setText("RoboVR Quest client\nConnecting to 127.0.0.1:7777 via adb reverse");
        statusView.setTextSize(24.0f);
        statusView.setPadding(48, 48, 48, 48);
        setContentView(statusView);
        nativeStart(this, "127.0.0.1", 7777);
    }

    @Override
    protected void onResume() {
        super.onResume();
        nativeOnResume();
    }

    @Override
    protected void onPause() {
        nativeOnPause();
        super.onPause();
    }

    @Override
    protected void onDestroy() {
        nativeStop();
        super.onDestroy();
    }
}
