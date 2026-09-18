#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONPATH="/usr/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}"
case "${1:-}" in
  probe|snapshot|capture)
    exec /usr/bin/python3 "$script_dir/mvs_capture.py" "$@"
    ;;
  preview)
    shift
    exec /usr/bin/python3 "$script_dir/live_preview.py" "$@"
    ;;
  gui)
    shift
    exec /usr/bin/python3 "$script_dir/mvs_gui.py" "$@"
    ;;
  mvs)
    shift
    mvs_root="${MVS_ROOT:-/home/ybbb/.local/opt/MVS-5.0.1}"
    export MVCAM_SDK_PATH="$mvs_root"
    export MVCAM_COMMON_RUNENV="$mvs_root/lib"
    export MVCAM_SOFTWARE_LIBENV="$mvs_root/lib"
    export MVCAM_GENICAM_CLPROTOCOL="$mvs_root/lib/CLProtocol"
    export LD_LIBRARY_PATH="$mvs_root/bin:$mvs_root/lib/64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    export QT_QPA_PLATFORM_PLUGIN_PATH="$mvs_root/bin/QtPlugins/platforms"
    cd "$mvs_root/bin"
    exec ./MVS -platform xcb "$@"
    ;;
  calibrate)
    shift
    exec /usr/bin/python3 "$script_dir/calibrate.py" "$@"
    ;;
  distance)
    shift
    exec /usr/bin/python3 "$script_dir/measure_board_distance.py" "$@"
    ;;
  *)
    echo "Usage: $0 {probe|snapshot|preview|gui|capture|calibrate|distance|mvs} [options]" >&2
    exit 2
    ;;
esac
