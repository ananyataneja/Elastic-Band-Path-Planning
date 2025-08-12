FROM gitpod/workspace-full

# Install JavaFX and VNC tools
USER root
RUN apt-get update && \
    apt-get install -y openjfx tigervnc-standalone-server novnc websockify && \
    apt-get clean
USER gitpod
