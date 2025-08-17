FROM gitpod/workspace-full

# Install JavaFX + desktop environment + VNC + noVNC
USER root
RUN apt-get update && \
    apt-get install -y openjfx xfce4 xfce4-terminal tigervnc-standalone-server novnc websockify && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

USER gitpod
