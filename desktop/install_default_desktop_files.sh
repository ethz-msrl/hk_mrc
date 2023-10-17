#!/bin/bash

# Set the source directories for desktop files and logos
DESKTOP_SRC_DIR="/home/$USER/navion_ws/src/Navion/desktop/default"
LOGO_SRC_DIR="/home/$USER/navion_ws/src/Navion/desktop/logos"

# Set the target directories for desktop files and icons in the user's home directory
DESKTOP_TARGET_DIR="$HOME/.local/share/applications"
ICON_TARGET_DIR="$HOME/.local/share/icons/hicolor/48x48/apps"

# Create the target directories if they don't exist
mkdir -p "$DESKTOP_TARGET_DIR"
mkdir -p "$ICON_TARGET_DIR"

# Copy the desktop files to the target directory and replace <username> with the current user's username
for file in "$DESKTOP_SRC_DIR"/*.desktop; do
    filename=$(basename "$file")
    sed "s|<username>|$USER|g" "$file" > "$DESKTOP_TARGET_DIR/$filename"
done

# Copy the logos to the target directory
cp -f "$LOGO_SRC_DIR"/*.png "$ICON_TARGET_DIR"

# Update the desktop database
update-desktop-database "$DESKTOP_TARGET_DIR"
