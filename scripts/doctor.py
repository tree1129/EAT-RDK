"""Read-only environment inventory. Does not configure hardware or install SDKs."""
import importlib.util
import json
from pathlib import Path
import platform
import sys


def read(path):
    p = Path(path)
    return p.read_text(errors='replace').strip('\x00\n') if p.is_file() else None


root = Path(__file__).resolve().parents[1]
print(json.dumps({'python':sys.version.split()[0], 'architecture':platform.machine(),
    'os_release':read('/etc/os-release'), 'board_model':read('/proc/device-tree/model'),
    'rdk_os_version':read('/etc/version'),
    'modules':{name:importlib.util.find_spec(name) is not None for name in ['numpy','cv2','pyrealsense2']},
    'vision_assets_present':all((root/x['path']).is_file() for x in json.loads((root/'scripts/vision-assets.json').read_text())),
    'hardware_execution_enabled':False,
    'note':'Environment inventory is not a camera capture or BPU compatibility test.'}, ensure_ascii=False, indent=2))
