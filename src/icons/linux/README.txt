Carla Studio - Linux icon set (hicolor theme)

Folder layout:
  icons/hicolor/scalable/apps/carla-studio.svg    Vector icon (primary)
  icons/hicolor/1024x1024/apps/carla-studio.png   High-res PNG fallback
  carla-studio.desktop                            Desktop entry
  install.sh                                      Installs to ~/.local/share

Qt integration:
  In your Qt app, set the window icon with
      QGuiApplication::setWindowIcon(QIcon::fromTheme("carla-studio"));
  after the icons are installed into a standard theme path.

To install system-wide instead, copy 'icons/hicolor' to /usr/share/icons/
and the .desktop file to /usr/share/applications/, then run:
  sudo gtk-update-icon-cache -f -t /usr/share/icons/hicolor
