#!/bin/sh
# Build a release freecad-rs.app in ./out (double-clickable macOS app).
set -e
cd "$(dirname "$0")/.."
cargo build --release -p fc-app
APP=out/freecad-rs.app
rm -rf "$APP"
mkdir -p "$APP/Contents/MacOS"
cp target/release/freecad-rs "$APP/Contents/MacOS/freecad-rs"
cat > "$APP/Contents/Info.plist" <<PLIST
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0"><dict>
  <key>CFBundleName</key><string>freecad-rs</string>
  <key>CFBundleIdentifier</key><string>com.fernando15suarez.freecad-rs</string>
  <key>CFBundleExecutable</key><string>freecad-rs</string>
  <key>CFBundlePackageType</key><string>APPL</string>
  <key>CFBundleVersion</key><string>0.1.0</string>
  <key>NSHighResolutionCapable</key><true/>
</dict></plist>
PLIST
echo "Built $APP"
