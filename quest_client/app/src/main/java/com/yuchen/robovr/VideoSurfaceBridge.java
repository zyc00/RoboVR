package com.yuchen.robovr;

import android.graphics.SurfaceTexture;
import android.view.Surface;

public final class VideoSurfaceBridge {
    private VideoSurfaceBridge() {
    }

    public static SurfaceTexture createSurfaceTexture(int textureName) {
        SurfaceTexture texture = new SurfaceTexture(textureName);
        texture.setDefaultBufferSize(1280, 1280);
        return texture;
    }

    public static Surface createSurface(SurfaceTexture texture) {
        return new Surface(texture);
    }

    public static void updateTexImage(SurfaceTexture texture) {
        texture.updateTexImage();
    }

    public static void releaseSurface(Surface surface) {
        surface.release();
    }

    public static void releaseSurfaceTexture(SurfaceTexture texture) {
        texture.release();
    }
}
