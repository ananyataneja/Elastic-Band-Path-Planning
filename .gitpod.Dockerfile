FROM gitpod/workspace-full

USER root

# Install only what we actually need: OpenJFX + VNC + minimal X11 + lightweight WM
RUN apt-get update && \
    apt-get install -y \
      openjfx \
      tigervnc-standalone-server \
      novnc \
      websockify \
      fluxbox \
      xterm \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

USER gitpod
