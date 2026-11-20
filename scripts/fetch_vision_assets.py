"""Download pinned optional MediaPipe assets with SHA-256 verification."""
import hashlib
import json
from pathlib import Path
import urllib.request

ROOT = Path(__file__).resolve().parents[1]


def main():
    manifest = json.loads((ROOT/'scripts/vision-assets.json').read_text())
    for item in manifest:
        target = ROOT/item['path']
        if target.is_file() and hashlib.sha256(target.read_bytes()).hexdigest() == item['sha256']:
            print('verified', item['path'])
            continue
        print('downloading', item['path'], flush=True)
        with urllib.request.urlopen(item['url'], timeout=60) as response:
            data = response.read(item['bytes']+1)
        if len(data) != item['bytes'] or hashlib.sha256(data).hexdigest() != item['sha256']:
            raise SystemExit('Asset checksum mismatch: '+item['path'])
        target.parent.mkdir(parents=True, exist_ok=True)
        temporary = target.with_suffix(target.suffix+'.download')
        temporary.write_bytes(data)
        temporary.replace(target)
    print('Vision assets ready; images are processed in your browser.')


if __name__ == '__main__':
    main()
